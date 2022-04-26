#!/usr/bin/env python3

import collections
import deepspeech
import os
import os.path
import pyaudio
import queue
import rospy
import string
import wave
import webrtcvad

from halo import Halo
import numpy as np
from pathlib import Path
from scipy import signal
from std_msgs.msg import String

ROS_NODE_NAME = "mozilla_deepspeech"
ROS_PUBLISHER_TOPIC_NAME = "recognized_text"
HOTWORDS_FILEPATH = Path(__file__).parent / "hotwords.csv"


class Audio(object):
    """Streams raw audio from microphone. Data is received in a separate thread, and stored in a buffer, to be read from."""

    FORMAT = pyaudio.paInt16
    # Network/VAD rate-space
    RATE_PROCESS = 16000
    CHANNELS = 1
    BLOCKS_PER_SECOND = 50

    frame_duration_ms = property(lambda self: 1000 * self.block_size // self.sample_rate)

    def __init__(self, callback=None, device=None, input_rate=RATE_PROCESS, file=None):
        def proxy_callback(in_data, frame_count, time_info, status):
            # pylint: disable=unused-argument
            if self.chunk is not None:
                in_data = self.wf.readframes(self.chunk)
            callback(in_data)
            return None, pyaudio.paContinue

        if callback is None: callback = lambda in_data: self.buffer_queue.put(in_data)
        self.buffer_queue = queue.Queue()
        self.device = device
        self.input_rate = input_rate
        self.sample_rate = self.RATE_PROCESS
        self.block_size = int(self.RATE_PROCESS / float(self.BLOCKS_PER_SECOND))
        self.block_size_input = int(self.input_rate / float(self.BLOCKS_PER_SECOND))
        self.pa = pyaudio.PyAudio()

        kwargs = {
            'format': self.FORMAT,
            'channels': self.CHANNELS,
            'rate': self.input_rate,
            'input': True,
            'frames_per_buffer': self.block_size_input,
            'stream_callback': proxy_callback,
        }

        self.chunk = None
        # if not default device
        if self.device:
            kwargs['input_device_index'] = self.device
        elif file is not None:
            self.chunk = 320
            self.wf = wave.open(file, 'rb')

        self.stream = self.pa.open(**kwargs)
        self.stream.start_stream()

    def resample(self, data, input_rate):
        """
        Microphone may not support our native processing sampling rate, so
        resample from input_rate to RATE_PROCESS here for webrtcvad and
        deepspeech

        Args:
            data (binary): Input audio stream
            input_rate (int): Input audio rate to resample from
        """
        data16 = np.fromstring(string=data, dtype=np.int16)
        resample_size = int(len(data16) / self.input_rate * self.RATE_PROCESS)
        resample = signal.resample(data16, resample_size)
        resample16 = np.array(resample, dtype=np.int16)
        return resample16.tostring()

    def read_resampled(self):
        """Return a block of audio data resampled to 16000hz, blocking if necessary."""
        return self.resample(data=self.buffer_queue.get(), input_rate=self.input_rate)

    def read(self):
        """Return a block of audio data, blocking if necessary."""
        return self.buffer_queue.get()

    def destroy(self):
        self.stream.stop_stream()
        self.stream.close()
        self.pa.terminate()


class VADAudio(Audio):
    """Filter & segment audio with voice activity detection."""

    def __init__(self, aggressiveness=3, device=None, input_rate=None, file=None):
        super(VADAudio, self).__init__(None, device, input_rate, file)
        self.vad = webrtcvad.Vad(aggressiveness)

    def frame_generator(self):
        """Generator that yields all audio frames from microphone."""
        if self.input_rate == self.RATE_PROCESS:
            while True:
                yield self.read()
        else:
            while True:
                yield self.read_resampled()

    def vad_collector(self, padding_ms=300, ratio=0.35, frames=None):
        """Generator that yields series of consecutive audio frames comprising each utterance, separated by yielding a single None.
            Determines voice activity by ratio of frames in padding_ms. Uses a buffer to include padding_ms prior to being triggered.
            Example: (frame, ..., frame, None, frame, ..., frame, None, ...)
                      |---utterance---|        |---utterance---|
        """
        if frames is None: frames = self.frame_generator()
        num_padding_frames = padding_ms // self.frame_duration_ms
        ring_buffer = collections.deque(maxlen=num_padding_frames)
        triggered = False

        for frame in frames:
            if len(frame) < 640:
                return

            is_speech = self.vad.is_speech(frame, self.sample_rate)

            if not triggered:
                ring_buffer.append((frame, is_speech))
                num_voiced = len([f for f, speech in ring_buffer if speech])
                if num_voiced > ratio * ring_buffer.maxlen:
                    triggered = True
                    for f, s in ring_buffer:
                        yield f
                    ring_buffer.clear()

            else:
                yield frame
                ring_buffer.append((frame, is_speech))
                num_unvoiced = len([f for f, speech in ring_buffer if not speech])
                if num_unvoiced > ratio * ring_buffer.maxlen:
                    triggered = False
                    yield None
                    ring_buffer.clear()


def clean_text(recognized_text):
    """ Cleans the transcribed text from isolated letters. The only one-letter words in English are 'a' and 'I'.

    :param recognized_text: the transcribed from the audio text
    :return: the transcribed text without single isolated letters
    """
    letters = list(string.ascii_lowercase)
    for letter in letters:
        if letter != 'a' and letter != 'i':
            recognized_text = recognized_text.replace(' ' + letter + ' ', ' ')

    return recognized_text


def main(args):
    # Initialize Ros Node and the Topic Publisher
    rospy.init_node(ROS_NODE_NAME)
    publisher = rospy.Publisher(ROS_PUBLISHER_TOPIC_NAME, String, queue_size=10)

    # Load DeepSpeech model
    if os.path.isdir(args.model):
        model_dir = args.model
        args.model = os.path.join(model_dir, 'output_graph.pb')
        args.scorer = os.path.join(model_dir, args.scorer)

    model = deepspeech.Model(args.model)
    if args.scorer:
        model.enableExternalScorer(args.scorer)

    # Add hot words, boost level can be (-inf, +inf)
    file = open(HOTWORDS_FILEPATH, "r")
    lines = file.readlines()
    for line in lines:
        hot_word, boost_value = line.split(",")
        model.addHotWord(hot_word, float(boost_value))

    # Start audio with VAD
    vad_audio = VADAudio(aggressiveness=args.vad_aggressiveness,
                         device=args.device,
                         input_rate=args.rate,
                         file=None)
    print("ROS node '%s' started. Listening for speech (ctrl-C to exit)..." % ROS_NODE_NAME)
    frames = vad_audio.vad_collector()

    # Stream from microphone to DeepSpeech using VAD
    spinner = None
    if not args.nospinner:
        spinner = Halo(spinner='line')
    stream_context = model.createStream()
    for frame in frames:
        if not rospy.is_shutdown():
            if frame is not None:
                if spinner: spinner.start()
                stream_context.feedAudioContent(np.frombuffer(frame, np.int16))
            else:
                if spinner: spinner.stop()
                recognized_text = stream_context.finishStream()
                if recognized_text:
                    recognized_text = clean_text(recognized_text)
                    print("Recognized: %s" % recognized_text)
                    publisher.publish(recognized_text)
                stream_context = model.createStream()
        else:
            stream_context.freeStream()
            print("Ctrl-C received. Shutting down ROS node '%s'!" % ROS_NODE_NAME)
            break


if __name__ == '__main__':
    DEFAULT_SAMPLE_RATE = 16000

    import argparse

    parser = argparse.ArgumentParser(description="Stream from microphone to DeepSpeech using VAD")

    parser.add_argument('-v', '--vad_aggressiveness', type=int, default=3,
                        help="Set aggressiveness of VAD: an integer between 0 and 3, 0 being the least aggressive about filtering out non-speech, "
                             "3 the most aggressive. Default: 3")
    parser.add_argument('--nospinner', action='store_true',
                        help="Disable spinner")
    parser.add_argument('-m', '--model',
                        help="Path to the model (protocol buffer binary file, or entire directory containing all standard-named files for model)")
    parser.add_argument('-s', '--scorer',
                        help="Path to the external scorer file.")
    parser.add_argument('-d', '--device', type=int, default=None,
                        help="Device input index (Int) as listed by pyaudio.PyAudio.get_device_info_by_index(). If not provided, falls back to "
                             "PyAudio.get_default_device().")
    parser.add_argument('-r', '--rate', type=int, default=DEFAULT_SAMPLE_RATE,
                        help="Input device sample rate. Default: {DEFAULT_SAMPLE_RATE}. Your device may require 48000.")

    args = parser.parse_args()

    try:
        main(args)
    except rospy.ROSInterruptException:
        pass
