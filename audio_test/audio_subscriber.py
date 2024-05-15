import rclpy
from rclpy.node import Node
from .util import ignore_stderr
from audio_common_msgs.msg import AudioData
import pyaudio
import speech_recognition as sr

class AudioSubscriber(Node):
    def __init__(self, suppress_error=True):
        super().__init__('audio_subscriber')
        self.subscription = self.create_subscription(
            AudioData, 'audio', self.listener_callback, 10)
         
        with ignore_stderr(enable=suppress_error): # ALSA error suppresion
            self.audio_interface = pyaudio.PyAudio()
            
        self.stream = self.audio_interface.open(format=pyaudio.paInt16,
                                                channels=1,
                                                rate=16000,
                                                output=True)
        self.recognizer = sr.Recognizer()
        self.audio_buffer = bytes()
        self.buffer_duration = 8  # seconds of a buffered audioclip
        self.bytes_per_frame = 2  # pyaudio.paInt16 -> 2 bytes per frame
        self.frame_rate = 16000  # rate
        self.buffer_size = self.buffer_duration * self.frame_rate * self.bytes_per_frame
        self.counter = 0

    def listener_callback(self, msg):
        self.counter += 1
        # appending data to buffer
        self.audio_buffer += bytes(msg.data)
        
        # if buffer has enough data to process
        if len(self.audio_buffer) >= self.buffer_size:
            print(self.counter)
            self.stream.write(self.audio_buffer)
            try:
                audio_data = sr.AudioData(self.audio_buffer, self.frame_rate, self.bytes_per_frame)
                text = self.recognizer.recognize_google(audio_data, language='et-EE')
                self.get_logger().info('Recognized Text: %s' % text)
            except sr.UnknownValueError:
                self.get_logger().info("Could not understand audio")
            except sr.RequestError as e:
                self.get_logger().info(f"Could not request results; {e}")
            except Exception as e:
                self.get_logger().info(f"Error during recognition; {str(e)}")
            finally:
                self.audio_buffer = bytes()  # clearing the buffer after processing

    def __del__(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio_interface.terminate()

def main(args=None):
    rclpy.init(args=args)

    audio_subscriber = AudioSubscriber()

    try:
        rclpy.spin(audio_subscriber)
    except KeyboardInterrupt:
        pass
    
    audio_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
