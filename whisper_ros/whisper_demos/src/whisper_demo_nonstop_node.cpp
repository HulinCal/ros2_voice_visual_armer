// MIT License
//
// Copyright (c) 2024 Miguel Ángel González Santamarta
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "whisper_msgs/action/stt.hpp"
#include "llama_ros_msgs/action/llama.hpp"
#include "paddle_speech_msgs/action/tts.hpp"

namespace whisper_demos {

class WhisperDemoNonstopNode : public rclcpp::Node {

public:
  using STT = whisper_msgs::action::STT;
  using GoalHandleSTT = rclcpp_action::ClientGoalHandle<STT>;
  using Llama = llama_ros_msgs::action::Llama;
  using GoalHandleLlama = rclcpp_action::ClientGoalHandle<Llama>;
  using TTS = paddle_speech_msgs::action::TTS;
  using GoalHandleTTS = rclcpp_action::ClientGoalHandle<TTS>;

  WhisperDemoNonstopNode();

private:
  void send_goal();
  void result_callback(const GoalHandleSTT::WrappedResult &result);
  void goal_response_callback(std::shared_ptr<GoalHandleSTT> goal_handle);
  void llama_goal_response_callback(GoalHandleLlama::SharedPtr goal_handle);
  void llama_result_callback(const GoalHandleLlama::WrappedResult &result);
  void tts_goal_response_callback(GoalHandleTTS::SharedPtr goal_handle);
  void tts_result_callback(const GoalHandleTTS::WrappedResult &result);

  rclcpp_action::Client<STT>::SharedPtr action_client_;
  rclcpp_action::Client<Llama>::SharedPtr llama_action_client_;
  rclcpp_action::Client<TTS>::SharedPtr tts_action_client_;

  bool llama_processing_;
};

WhisperDemoNonstopNode::WhisperDemoNonstopNode()
    : Node("whisper_demo_nonstop_node"),
      llama_processing_(false) {

  this->action_client_ = rclcpp_action::create_client<STT>(this, "/whisper/listen");
  this->llama_action_client_ = rclcpp_action::create_client<Llama>(this, "llama_inference");
  this->tts_action_client_ = rclcpp_action::create_client<TTS>(this, "text_to_speech");

  RCLCPP_INFO(this->get_logger(), "Listening...");

  this->send_goal();
}

void WhisperDemoNonstopNode::send_goal() {
  if (this->llama_processing_) {
    RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] Llama is processing, waiting...");
    return;
  }

  auto goal = STT::Goal();

  if (!this->action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(this->get_logger(), "[WhisperDemoNonstopNode] Action server not available");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] SPEAK - Send goal to whisper server");

  auto send_goal_options = rclcpp_action::Client<STT>::SendGoalOptions();
  send_goal_options.result_callback = std::bind(&WhisperDemoNonstopNode::result_callback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback = std::bind(&WhisperDemoNonstopNode::goal_response_callback, this, std::placeholders::_1);

  this->action_client_->async_send_goal(goal, send_goal_options);
}

void WhisperDemoNonstopNode::goal_response_callback(std::shared_ptr<GoalHandleSTT> goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "[WhisperDemoNonstopNode] Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] Goal accepted by server, waiting for result...");
  }
}

void WhisperDemoNonstopNode::llama_goal_response_callback(GoalHandleLlama::SharedPtr goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "[WhisperDemoNonstopNode] Llama goal was rejected by server");
    this->llama_processing_ = false;
  } else {
    RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] Llama goal accepted by server, waiting for result...");
  }
}

void WhisperDemoNonstopNode::tts_goal_response_callback(GoalHandleTTS::SharedPtr goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "[WhisperDemoNonstopNode] TTS goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] TTS goal accepted by server, waiting for result...");
  }
}

void WhisperDemoNonstopNode::tts_result_callback(const GoalHandleTTS::WrappedResult &result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] TTS playback completed: %s", result.result->message.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "[WhisperDemoNonstopNode] TTS goal was canceled or failed");
  }
  
  RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] TTS playback finished, resuming speech recognition...");
  this->send_goal();
}

void WhisperDemoNonstopNode::llama_result_callback(const GoalHandleLlama::WrappedResult &result) {
  this->llama_processing_ = false;
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    std::string response = result.result->response;
    RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] Llama response: %s", response.c_str());

    if (!response.empty()) {
      if (!this->tts_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "[WhisperDemoNonstopNode] TTS action server not available");
        this->send_goal();
        return;
      }

      auto tts_goal = TTS::Goal();
      tts_goal.text = response;

      auto send_goal_options = rclcpp_action::Client<TTS>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&WhisperDemoNonstopNode::tts_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.result_callback = std::bind(&WhisperDemoNonstopNode::tts_result_callback, this, std::placeholders::_1);

      RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] Sending response to TTS server: %s", response.c_str());
      this->tts_action_client_->async_send_goal(tts_goal, send_goal_options);
    } else {
      this->send_goal();
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "[WhisperDemoNonstopNode] Llama goal was canceled or failed");
    this->send_goal();
  }
}

void WhisperDemoNonstopNode::result_callback(const GoalHandleSTT::WrappedResult &result) {
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    auto transcription = result.result->transcription;
    RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] I hear: %s", transcription.text.c_str());
    RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] Audio time: %.2f", transcription.audio_time);
    RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] Transcription time: %.2f", transcription.transcription_time);

    if (!transcription.text.empty()) {
      if (!this->llama_action_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "[WhisperDemoNonstopNode] Llama action server not available");
        return;
      }

      this->llama_processing_ = true;
      auto llama_goal = Llama::Goal();
      llama_goal.text = transcription.text;

      auto send_goal_options = rclcpp_action::Client<Llama>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&WhisperDemoNonstopNode::llama_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.result_callback = std::bind(&WhisperDemoNonstopNode::llama_result_callback, this, std::placeholders::_1);

      RCLCPP_INFO(this->get_logger(), "[WhisperDemoNonstopNode] Sending text to Llama server: %s", transcription.text.c_str());
      this->llama_action_client_->async_send_goal(llama_goal, send_goal_options);
    } else {
      this->send_goal();
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "[WhisperDemoNonstopNode] Goal was canceled or failed");
    if (!this->llama_processing_) {
      this->send_goal();
    }
  }
}

} // namespace whisper_demos

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<whisper_demos::WhisperDemoNonstopNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
