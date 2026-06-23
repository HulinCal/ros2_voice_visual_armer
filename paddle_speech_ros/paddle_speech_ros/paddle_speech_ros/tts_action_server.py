import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
import threading
import queue
import tempfile
import os

from paddle_speech_msgs.action import TTS


class TTSActionServer(Node):

    def __init__(self):
        super().__init__('tts_action_server')
        self._action_server = ActionServer(
            self,
            TTS,
            'text_to_speech',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('TTS Action Server started')
        self.is_shutdown = False

        self.get_logger().info('Loading FastSpeech2 ONNX model...')
        from paddlespeech.cli.tts import TTSExecutor
        self.tts_executor = TTSExecutor()

        self.get_logger().info('Warming up TTS model...')
        self.warmup_tts()
        self.get_logger().info('TTS warmup completed - Ready for synthesis')

    def warmup_tts(self):
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                temp_wav = f.name

            self.tts_executor(
                text='测试！',
                am='fastspeech2_csmsc',
                voc='hifigan_csmsc',
                use_onnx=True,
                output=temp_wav
            )

            if os.path.exists(temp_wav):
                os.remove(temp_wav)

            self.get_logger().info('TTS warmup successful')
        except Exception as e:
            self.get_logger().error(f'TTS warmup failed: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received TTS request: {goal_request.text}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def synthesize_and_play(self, text):
        try:
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                temp_wav = f.name

            self.get_logger().info(f'Generating speech for: "{text}"')
            self.tts_executor(
                text=text,
                am='fastspeech2_csmsc',
                voc='hifigan_csmsc',
                use_onnx=True,
                output=temp_wav
            )

            self.get_logger().info(f'Reading audio file...')
            import soundfile as sf
            import sounddevice as sd
            data, samplerate = sf.read(temp_wav)
            os.remove(temp_wav)

            self.get_logger().info(f'Playing audio...')
            sd.play(data, samplerate)
            sd.wait()

            audio_duration = len(data) / samplerate
            self.get_logger().info(f'Playback completed, duration: {audio_duration:.2f}s')

            return True, audio_duration

        except Exception as e:
            self.get_logger().error(f'TTS error: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            return False, 0.0

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('Executing TTS request')

        text = goal_handle.request.text
        feedback = TTS.Feedback()
        result = TTS.Result()

        try:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Canceled'
                result.duration = 0.0
                return result

            feedback.progress = 0.3
            goal_handle.publish_feedback(feedback)

            success, audio_duration = self.synthesize_and_play(text)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Canceled'
                result.duration = 0.0
                return result

            feedback.progress = 1.0
            goal_handle.publish_feedback(feedback)

            if success:
                goal_handle.succeed()
                result.success = True
                result.message = f'Successfully synthesized and played speech with duration {audio_duration:.2f}s'
                result.duration = audio_duration
            else:
                goal_handle.abort()
                result.success = False
                result.message = 'TTS synthesis failed'
                result.duration = 0.0

            return result

        except Exception as e:
            self.get_logger().error(f'TTS execute error: {str(e)}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')
            goal_handle.abort()
            result.success = False
            result.message = str(e)
            result.duration = 0.0
            return result


def main(args=None):
    rclpy.init(args=args)
    tts_action_server = TTSActionServer()
    rclpy.spin(tts_action_server)
    tts_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
