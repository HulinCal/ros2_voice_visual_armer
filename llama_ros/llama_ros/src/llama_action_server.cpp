#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <fstream>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <algorithm>
#include <future>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include "llama.h"
#include "llama-cpp.h"
#include "mtmd.h"
#include "mtmd-helper.h"

#include "llama_ros_msgs/action/llama.hpp"
#include "arm_controller/action/arm_interface.hpp"

using LlamaAction = llama_ros_msgs::action::Llama;
using GoalHandleLlama = rclcpp_action::ServerGoalHandle<LlamaAction>;

using ArmInterface = arm_controller::action::ArmInterface;
using GoalHandleArmInterface = rclcpp_action::ClientGoalHandle<ArmInterface>;

enum class Intent {
    GENERAL_CHAT,
    IMAGE_CHAT,
    COMMAND
};

class LlamaActionServer : public rclcpp::Node {
public:
    LlamaActionServer()
        : Node("llama_action_server"),
          processing_(false),
          latest_image_(nullptr),
          image_mutex_(std::make_shared<std::mutex>()) {

        RCLCPP_INFO(this->get_logger(), "Llama Action Server started");

        action_server_ = rclcpp_action::create_server<LlamaAction>(
            this,
            "llama_inference",
            std::bind(&LlamaActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&LlamaActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&LlamaActionServer::handle_accepted, this, std::placeholders::_1));

        publisher_ = this->create_publisher<std_msgs::msg::String>("text_to_speech/input", 10);

        arm_action_client_ = rclcpp_action::create_client<ArmInterface>(
            this,
            "arm_interface");

        RCLCPP_INFO(this->get_logger(), "Arm action client created, waiting for server...");
        if (!arm_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "Arm action server not available within 5 seconds");
        } else {
            RCLCPP_INFO(this->get_logger(), "Arm action server connected");
        }

        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(*image_mutex_);
                latest_image_ = msg;
                RCLCPP_DEBUG(this->get_logger(), "Received new image: %ux%u", msg->width, msg->height);
            });

        llama_backend_init();

        const char * model_path = "/usr/models/Qwen2.5-VL-7B-Instruct-IQ4_XS.gguf";
        const char * mmproj_path = "/usr/models/mmproj-F16.gguf";

        auto model_params = llama_model_default_params();
        model_params.n_gpu_layers = 99;

        RCLCPP_INFO(this->get_logger(), "Loading model from: %s", model_path);
        model_.reset(llama_model_load_from_file(model_path, model_params));
        if (!model_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load model");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Model loaded successfully");

        auto ctx_params = llama_context_default_params();
        ctx_params.n_ctx = 4096;
        ctx_params.n_batch = 512;
        ctx_params.n_threads = 4;
        ctx_params.n_threads_batch = 4;

        RCLCPP_INFO(this->get_logger(), "Creating context");
        ctx_.reset(llama_init_from_model(model_.get(), ctx_params));
        if (!ctx_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create context");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Context created successfully");

        auto mtmd_params = mtmd_context_params_default();
        mtmd_params.use_gpu = true;
        mtmd_params.n_threads = 4;
        mtmd_params.warmup = true;

        RCLCPP_INFO(this->get_logger(), "Initializing multimodal context from: %s", mmproj_path);
        mtmd_ctx_.reset(mtmd_init_from_file(mmproj_path, model_.get(), mtmd_params));
        if (!mtmd_ctx_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize multimodal context");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Multimodal context initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Llama Action Server ready!");
    }

private:
    rclcpp_action::Server<LlamaAction>::SharedPtr action_server_;
    rclcpp_action::Client<ArmInterface>::SharedPtr arm_action_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    sensor_msgs::msg::Image::SharedPtr latest_image_;
    std::shared_ptr<std::mutex> image_mutex_;

    llama_model_ptr model_;
    llama_context_ptr ctx_;
    mtmd::context_ptr mtmd_ctx_;

    std::atomic<bool> processing_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const LlamaAction::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with text: %s", goal->text.c_str());
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleLlama> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleLlama> goal_handle) {
        if (processing_.exchange(true)) {
            RCLCPP_WARN(this->get_logger(), "Already processing, rejecting new request");
            goal_handle->abort(std::make_shared<LlamaAction::Result>());
            processing_ = false;
            return;
        }

        std::thread{std::bind(&LlamaActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleLlama> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        auto feedback = std::make_shared<LlamaAction::Feedback>();
        auto result = std::make_shared<LlamaAction::Result>();

        std::string user_input = goal_handle->get_goal()->text;
        std::string full_response;
        std::string command_info;

        RCLCPP_INFO(this->get_logger(), "User input: %s", user_input.c_str());

        feedback->progress = 0.0;
        feedback->status = "Analyzing intent";
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Starting intent classification...");
        Intent intent = classify_intent(user_input, goal_handle);
        RCLCPP_INFO(this->get_logger(), "Intent classification completed");

        if (intent == Intent::COMMAND) {
            command_info = parse_command(user_input);
            RCLCPP_INFO(this->get_logger(), "Parsed command info: %s", command_info.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Switching based on intent: %d", static_cast<int>(intent));

        switch (intent) {
            case Intent::GENERAL_CHAT:
                RCLCPP_INFO(this->get_logger(), "Intent classified: GENERAL_CHAT - performing text-only inference");
                full_response = run_text_only_inference(user_input, goal_handle);
                break;

            case Intent::IMAGE_CHAT:
                RCLCPP_INFO(this->get_logger(), "Intent classified: IMAGE_CHAT - performing multimodal inference");
                full_response = run_multimodal_chat(user_input, goal_handle);
                break;

            case Intent::COMMAND:
                RCLCPP_INFO(this->get_logger(), "Intent classified: COMMAND - processing command: %s", command_info.c_str());
                full_response = run_arm_command(command_info, goal_handle);
                break;
        }

        RCLCPP_INFO(this->get_logger(), "Response generated: %s", full_response.c_str());

        feedback->progress = 1.0;
        feedback->status = "Completed";
        goal_handle->publish_feedback(feedback);

        result->response = full_response;
        result->success = true;
        result->message = "Inference completed successfully";

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");

        processing_ = false;
    }

    Intent classify_intent(const std::string& user_input, const std::shared_ptr<GoalHandleLlama> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Classifying intent for: %s", user_input.c_str());

        std::string intent_prompt =
            "<|im_start|>system\n"
            "你是一个意图分类助手。请用中文回答。根据用户的输入，将意图分类为以下三类之一：\n"
            "1. GENERAL_CHAT: 普通对话聊天，不需要查看图片。\n"
            "   示例: '你好', '今天天气怎么样', '给我讲个笑话', '你是谁'\n"
            "2. IMAGE_CHAT: 用户想讨论、描述或询问视觉相关的内容。\n"
            "   示例: '描述一下你看到了什么', '图片里有什么', '这个东西看起来怎么样', '看到了什么感想', '这张图片有意思吗'\n"
            "3. COMMAND: 用户想控制机器人或系统。\n"
            "   示例命令: '往前走', '停下来', '转向左边', '把机械臂升高', '抓紧', '松开', '移动苹果', '把苹果放到碟子里', '拿起方块'\n"
            "请只回答一个词: GENERAL_CHAT、IMAGE_CHAT 或 COMMAND\n"
            "<|im_end|>\n"
            "<|im_start|>user\n"
            + user_input + "\n"
            "<|im_end|>\n"
            "<|im_start|>assistant\n";

        RCLCPP_INFO(this->get_logger(), "Running intent classification prompt");

        llama_memory_t mem = llama_get_memory(ctx_.get());
        llama_memory_seq_rm(mem, 0, -1, -1);

        std::vector<llama_token> prompt_tokens;
        prompt_tokens.resize(intent_prompt.size() + 256);

        int32_t n_tokens = llama_tokenize(llama_model_get_vocab(model_.get()), intent_prompt.c_str(), intent_prompt.size(),
                                           prompt_tokens.data(), prompt_tokens.size(), true, true);
        if (n_tokens < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to tokenize intent prompt: %d", n_tokens);
            return Intent::GENERAL_CHAT;
        }

        prompt_tokens.resize(n_tokens);

        for (int i = 0; i < n_tokens; i++) {
            llama_batch batch = llama_batch_init(1, 0, 1);
            batch.n_tokens = 1;
            batch.token[0] = prompt_tokens[i];
            batch.pos[0] = i;
            batch.n_seq_id[0] = 1;
            batch.seq_id[0] = (llama_seq_id *)malloc(sizeof(llama_seq_id));
            batch.seq_id[0][0] = 0;
            batch.logits[0] = 1;

            if (llama_decode(ctx_.get(), batch)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode token %d", i);
                llama_batch_free(batch);
                return Intent::GENERAL_CHAT;
            }
            llama_batch_free(batch);
        }

        auto sparams = llama_sampler_chain_default_params();
        llama_sampler * smpl = llama_sampler_chain_init(sparams);
        llama_sampler_chain_add(smpl, llama_sampler_init_top_k(10));
        llama_sampler_chain_add(smpl, llama_sampler_init_top_p(0.9, 1));
        llama_sampler_chain_add(smpl, llama_sampler_init_temp(0.3));
        llama_sampler_chain_add(smpl, llama_sampler_init_dist(0));

        std::string intent_result;
        llama_token new_token;
        int n_eval = 0;
        int max_tokens = 20;

        while (n_eval < max_tokens) {
            new_token = llama_sampler_sample(smpl, ctx_.get(), -1);

            if (llama_vocab_is_eog(llama_model_get_vocab(model_.get()), new_token)) {
                break;
            }

            char buf[256];
            int n = llama_token_to_piece(llama_model_get_vocab(model_.get()), new_token, buf, sizeof(buf), 0, true);
            if (n > 0) {
                intent_result.append(buf, n);
            }

            llama_batch batch = llama_batch_init(1, 0, 1);
            batch.n_tokens = 1;
            batch.token[0] = new_token;
            batch.pos[0] = n_tokens;
            batch.n_seq_id[0] = 1;
            batch.seq_id[0] = (llama_seq_id *)malloc(sizeof(llama_seq_id));
            batch.seq_id[0][0] = 0;
            batch.logits[0] = 1;

            if (llama_decode(ctx_.get(), batch)) {
                llama_batch_free(batch);
                break;
            }
            llama_batch_free(batch);

            n_tokens++;
            n_eval++;
        }

        llama_sampler_free(smpl);

        RCLCPP_INFO(this->get_logger(), "Intent classification result: %s", intent_result.c_str());

        if (intent_result.find("IMAGE_CHAT") != std::string::npos) {
            return Intent::IMAGE_CHAT;
        } else if (intent_result.find("COMMAND") != std::string::npos) {
            return Intent::COMMAND;
        }

        return Intent::GENERAL_CHAT;
    }

    std::string parse_command(const std::string& user_input) {
        std::string lower_input = user_input;
        std::transform(lower_input.begin(), lower_input.end(), lower_input.begin(), ::tolower);

        // 停止命令 - 包含相似读音
        if (lower_input.find("停止") != std::string::npos ||
            lower_input.find("停下") != std::string::npos ||
            lower_input.find("停下来") != std::string::npos ||
            lower_input.find("停") != std::string::npos ||
            lower_input.find("听") != std::string::npos ||
            lower_input.find("挺") != std::string::npos ||
            lower_input.find("厅") != std::string::npos ||
            lower_input.find("艇") != std::string::npos ||
            lower_input.find("停滞") != std::string::npos ||
            lower_input.find("停职") != std::string::npos ||
            lower_input.find("挺住") != std::string::npos ||
            lower_input.find("挺止") != std::string::npos ||
            lower_input.find("停止吧") != std::string::npos ||
            lower_input.find("停一下") != std::string::npos ||
            lower_input.find("暂停") != std::string::npos ||
            lower_input.find("休止") != std::string::npos ||
            lower_input.find("停息") != std::string::npos) {
            return "stop";
        }

        // 恢复/复原命令
        if (lower_input.find("恢复") != std::string::npos ||
            lower_input.find("复原") != std::string::npos ||
            lower_input.find("回复") != std::string::npos ||
            lower_input.find("恢复原位") != std::string::npos ||
            lower_input.find("回到原位") != std::string::npos ||
            lower_input.find("归位") != std::string::npos) {
            return "recover";
        }

        // 右转命令 - 包含相似读音和长句意图
        if (lower_input.find("右转") != std::string::npos ||
            lower_input.find("向右转") != std::string::npos ||
            lower_input.find("请右转") != std::string::npos ||
            lower_input.find("右移动") != std::string::npos ||
            lower_input.find("右传") != std::string::npos ||
            lower_input.find("右赚") != std::string::npos ||
            lower_input.find("右专") != std::string::npos ||
            lower_input.find("往右转") != std::string::npos ||
            lower_input.find("向右") != std::string::npos ||
            lower_input.find("转到右边") != std::string::npos ||
            lower_input.find("转到右侧") != std::string::npos ||
            lower_input.find("右边转") != std::string::npos ||
            lower_input.find("右移") != std::string::npos ||
            lower_input.find("向右移") != std::string::npos ||
            lower_input.find("往右") != std::string::npos) {
            return "turnright";
        }

        // 左转命令 - 包含相似读音和长句意图
        if (lower_input.find("左转") != std::string::npos ||
            lower_input.find("向左转") != std::string::npos ||
            lower_input.find("请左转") != std::string::npos ||
            lower_input.find("左移动") != std::string::npos ||
            lower_input.find("左传") != std::string::npos ||
            lower_input.find("左赚") != std::string::npos ||
            lower_input.find("左专") != std::string::npos ||
            lower_input.find("往左转") != std::string::npos ||
            lower_input.find("向左") != std::string::npos ||
            lower_input.find("转到左边") != std::string::npos ||
            lower_input.find("转到左侧") != std::string::npos ||
            lower_input.find("左边转") != std::string::npos ||
            lower_input.find("左移") != std::string::npos ||
            lower_input.find("向左移") != std::string::npos ||
            lower_input.find("往左") != std::string::npos) {
            return "turnleft";
        }

        // 降低命令 - 包含相似读音和长句意图
        if (lower_input.find("降低") != std::string::npos ||
            lower_input.find("减低") != std::string::npos ||
            lower_input.find("请降低") != std::string::npos ||
            lower_input.find("下降") != std::string::npos ||
            lower_input.find("降下") != std::string::npos ||
            lower_input.find("放低") != std::string::npos ||
            lower_input.find("调低") != std::string::npos ||
            lower_input.find("低一点") != std::string::npos ||
            lower_input.find("降低一点") != std::string::npos ||
            lower_input.find("把机械臂降低") != std::string::npos ||
            lower_input.find("把机械臂放低") != std::string::npos ||
            lower_input.find("机械臂降低") != std::string::npos ||
            lower_input.find("手臂降低") != std::string::npos ||
            lower_input.find("手臂放低") != std::string::npos ||
            lower_input.find("往下") != std::string::npos ||
            lower_input.find("向下") != std::string::npos) {
            return "lower";
        }

        // 升高命令 - 包含相似读音和长句意图
        if (lower_input.find("升高") != std::string::npos ||
            lower_input.find("提起") != std::string::npos ||
            lower_input.find("提高") != std::string::npos ||
            lower_input.find("抬起来") != std::string::npos ||
            lower_input.find("抬起") != std::string::npos ||
            lower_input.find("举起来") != std::string::npos ||
            lower_input.find("举起") != std::string::npos ||
            lower_input.find("升起来") != std::string::npos ||
            lower_input.find("高一点") != std::string::npos ||
            lower_input.find("升高一点") != std::string::npos ||
            lower_input.find("把机械臂升高") != std::string::npos ||
            lower_input.find("把机械臂提高") != std::string::npos ||
            lower_input.find("机械臂升高") != std::string::npos ||
            lower_input.find("手臂升高") != std::string::npos ||
            lower_input.find("手臂提高") != std::string::npos ||
            lower_input.find("往上") != std::string::npos ||
            lower_input.find("向上") != std::string::npos ||
            lower_input.find("提起来") != std::string::npos ||
            lower_input.find("拎起来") != std::string::npos) {
            return "higher";
        }

        // 向前命令 - 包含相似读音和长句意图
        if (lower_input.find("向前") != std::string::npos ||
            lower_input.find("往前走") != std::string::npos ||
            lower_input.find("前进") != std::string::npos ||
            lower_input.find("前移") != std::string::npos ||
            lower_input.find("向前走") != std::string::npos ||
            lower_input.find("往前") != std::string::npos ||
            lower_input.find("向前移动") != std::string::npos ||
            lower_input.find("往前移动") != std::string::npos ||
            lower_input.find("把机械臂向前") != std::string::npos ||
            lower_input.find("机械臂向前") != std::string::npos ||
            lower_input.find("手臂向前") != std::string::npos ||
            lower_input.find("向前移") != std::string::npos ||
            lower_input.find("前移一点") != std::string::npos ||
            lower_input.find("向前一点") != std::string::npos) {
            return "forward";
        }

        // 向后命令 - 包含相似读音和长句意图
        if (lower_input.find("向后") != std::string::npos ||
            lower_input.find("往后") != std::string::npos ||
            lower_input.find("后退") != std::string::npos ||
            lower_input.find("后移") != std::string::npos ||
            lower_input.find("向后退") != std::string::npos ||
            lower_input.find("往后退") != std::string::npos ||
            lower_input.find("向后移动") != std::string::npos ||
            lower_input.find("往后移动") != std::string::npos ||
            lower_input.find("把机械臂向后") != std::string::npos ||
            lower_input.find("机械臂向后") != std::string::npos ||
            lower_input.find("手臂向后") != std::string::npos ||
            lower_input.find("向后移") != std::string::npos ||
            lower_input.find("后移一点") != std::string::npos ||
            lower_input.find("向后一点") != std::string::npos) {
            return "backward";
        }

        // 抓紧命令 - 包含相似读音和长句意图
        if (lower_input.find("抓紧") != std::string::npos ||
            lower_input.find("夹住") != std::string::npos ||
            lower_input.find("抓住") != std::string::npos ||
            lower_input.find("抓取") != std::string::npos ||
            lower_input.find("夹紧") != std::string::npos ||
            lower_input.find("握住") != std::string::npos ||
            lower_input.find("握紧") != std::string::npos ||
            lower_input.find("抓佳") != std::string::npos ||
            lower_input.find("抓家") != std::string::npos ||
            lower_input.find("夹住它") != std::string::npos ||
            lower_input.find("抓住它") != std::string::npos ||
            lower_input.find("把它夹住") != std::string::npos ||
            lower_input.find("把它抓住") != std::string::npos ||
            lower_input.find("夹爪抓紧") != std::string::npos ||
            lower_input.find("夹爪夹住") != std::string::npos ||
            lower_input.find("夹爪抓住") != std::string::npos ||
            lower_input.find("夹紧一点") != std::string::npos ||
            lower_input.find("抓紧一点") != std::string::npos) {
            return "grip";
        }

        // 松开命令 - 包含相似读音和长句意图
        if (lower_input.find("松开") != std::string::npos ||
            lower_input.find("放开") != std::string::npos ||
            lower_input.find("松手") != std::string::npos ||
            lower_input.find("释放") != std::string::npos ||
            lower_input.find("松开它") != std::string::npos ||
            lower_input.find("放开它") != std::string::npos ||
            lower_input.find("把它松开") != std::string::npos ||
            lower_input.find("把它放开") != std::string::npos ||
            lower_input.find("夹爪松开") != std::string::npos ||
            lower_input.find("夹爪放开") != std::string::npos ||
            lower_input.find("松开夹爪") != std::string::npos ||
            lower_input.find("放开夹爪") != std::string::npos ||
            lower_input.find("松开一点") != std::string::npos ||
            lower_input.find("放开一点") != std::string::npos ||
            lower_input.find("松宽") != std::string::npos ||
            lower_input.find("放宽") != std::string::npos) {
            return "loose";
        }

        // 移动苹果
        if (lower_input.find("苹果") != std::string::npos ||
            lower_input.find("平果") != std::string::npos ||
            lower_input.find("频果") != std::string::npos) {
            return "move_apple";
        }

        // 移动方块 - 包含相似读音
        if (lower_input.find("方块") != std::string::npos ||
            lower_input.find("方快") != std::string::npos ||
            lower_input.find("方块子") != std::string::npos ||
            lower_input.find("方块儿") != std::string::npos) {
            return "move_cube";
        }

        // 移动杯子 - 包含相似读音
        if (lower_input.find("杯子") != std::string::npos ||
            lower_input.find("杯紫") != std::string::npos ||
            lower_input.find("杯字") != std::string::npos ||
            lower_input.find("杯儿") != std::string::npos) {
            return "move_cup";
        }

        // 移动瓶子 - 包含相似读音
        if (lower_input.find("瓶子") != std::string::npos ||
            lower_input.find("瓶紫") != std::string::npos ||
            lower_input.find("瓶字") != std::string::npos ||
            lower_input.find("瓶儿") != std::string::npos) {
            return "move_bottle";
        }

        // 打开命令
        if (lower_input.find("打开") != std::string::npos ||
            lower_input.find("开启") != std::string::npos ||
            lower_input.find("开") != std::string::npos) {
            return "open";
        }

        // 关闭命令
        if (lower_input.find("关闭") != std::string::npos ||
            lower_input.find("关上") != std::string::npos ||
            lower_input.find("关") != std::string::npos ||
            lower_input.find("合上") != std::string::npos) {
            return "close";
        }

        return "unknown_command";
    }

    std::string run_arm_command(const std::string& command_info, const std::shared_ptr<GoalHandleLlama> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing arm command via action client: %s", command_info.c_str());

        auto feedback = std::make_shared<LlamaAction::Feedback>();
        feedback->progress = 0.3;
        feedback->status = "Processing command";
        feedback->partial_response = "正在执行命令: " + command_info;
        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(this->get_logger(), "Checking if arm action server is available...");
        if (!arm_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Arm action server NOT available, cannot send command");
            return "手臂控制服务不可用，请稍后重试。";
        }
        RCLCPP_INFO(this->get_logger(), "Arm action server is available, preparing to send command...");

        std::promise<std::pair<bool, std::string>> result_promise;
        auto result_future = result_promise.get_future();

        auto arm_goal = std::make_shared<ArmInterface::Goal>();
        arm_goal->command = command_info;

        auto send_goal_options = rclcpp_action::Client<ArmInterface>::SendGoalOptions();
        
        send_goal_options.feedback_callback =
            [this, goal_handle](std::shared_ptr<GoalHandleArmInterface> arm_goal_handle,
                               const std::shared_ptr<const ArmInterface::Feedback> arm_feedback) {
                auto llama_feedback = std::make_shared<LlamaAction::Feedback>();
                llama_feedback->progress = 0.3 + arm_feedback->progress * 0.7;
                llama_feedback->status = "Executing arm command";
                llama_feedback->partial_response = arm_feedback->status + ": " + arm_feedback->partial_result;
                goal_handle->publish_feedback(llama_feedback);
                RCLCPP_INFO(this->get_logger(), "Arm feedback: %s (progress: %.2f)", 
                           arm_feedback->status.c_str(), arm_feedback->progress);
            };

        send_goal_options.result_callback =
            [&result_promise](const GoalHandleArmInterface::WrappedResult& result) {
                bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
                std::string msg = result.result->message;
                result_promise.set_value(std::make_pair(success, msg));
            };

        std::thread arm_thread([this, arm_goal, &send_goal_options, &result_promise]() {
            try {
                auto send_future = this->arm_action_client_->async_send_goal(*arm_goal, send_goal_options);
                
                if (send_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to send arm action goal in thread");
                    result_promise.set_value(std::make_pair(false, "Failed to send goal"));
                    return;
                }

                auto goal_handle_arm = send_future.get();
                if (!goal_handle_arm) {
                    RCLCPP_ERROR(this->get_logger(), "Arm action goal was rejected in thread");
                    result_promise.set_value(std::make_pair(false, "Goal rejected"));
                    return;
                }

                this->arm_action_client_->async_get_result(goal_handle_arm);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in arm thread: %s", e.what());
                result_promise.set_value(std::make_pair(false, std::string("Exception: ") + e.what()));
            }
        });

        arm_thread.detach();

        if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get arm action result");
            return "获取命令执行结果失败。";
        }

        auto [success, result_msg] = result_future.get();

        if (success) {
            feedback->progress = 1.0;
            feedback->status = "Command executed";
            goal_handle->publish_feedback(feedback);

            std::string response;
            if (command_info == "forward") {
                response = "";
            } else if (command_info == "backward") {
                response = "";
            } else if (command_info == "turnleft") {
                response = "";
            } else if (command_info == "turnright") {
                response = "";
            } else if (command_info == "stop") {
                response = "";
            } else if (command_info == "recover") {
                response = "";
            } else if (command_info == "lower") {
                response = "";
            } else if (command_info == "higher") {
                response = "";
            } else if (command_info == "grip") {
                response = "";
            } else if (command_info == "loose") {
                response = "";
            } else if (command_info == "move_apple") {
                response = "移动苹果完成";
            } else if (command_info == "move_cube") {
                response = "移动方块完成";
            } else if (command_info == "move_cup") {
                response = "移动杯子完成";
            } else if (command_info == "move_bottle") {
                response = "移动瓶子完成";
            } else if (command_info == "open") {
                response = "打开完成";
            } else if (command_info == "close") {
                response = "关闭完成";
            } else {
                response = "命令执行完成";
            }
            return response;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Arm command failed: %s", result_msg.c_str());
            return "命令执行失败: " + result_msg;
        }
    }

    std::string run_multimodal_chat(const std::string& user_input, 
                                    const std::shared_ptr<GoalHandleLlama> goal_handle) {
        std::lock_guard<std::mutex> lock(*image_mutex_);

        if (!latest_image_) {
            RCLCPP_WARN(this->get_logger(), "No image available, falling back to text-only inference");
            return run_text_only_inference(user_input, goal_handle);
        }

        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*latest_image_, sensor_msgs::image_encodings::BGR8);
            return run_multimodal_inference_from_image(cv_ptr->image, user_input, goal_handle);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to convert image: %s", e.what());
            return "抱歉，图像处理出现问题。";
        }
    }

    std::string run_multimodal_inference_from_image(const cv::Mat& image, const std::string& prompt, 
                                                    const std::shared_ptr<GoalHandleLlama> goal_handle) {
        llama_memory_t mem = llama_get_memory(ctx_.get());
        llama_memory_seq_rm(mem, 0, -1, -1);
        RCLCPP_INFO(this->get_logger(), "Running multimodal inference with image: %dx%d", 
                    image.cols, image.rows);

        cv::Mat rgb_image;
        cv::cvtColor(image, rgb_image, cv::COLOR_BGR2RGB);

        uint32_t nx = rgb_image.cols;
        uint32_t ny = rgb_image.rows;
        
        std::vector<unsigned char> rgb_data(rgb_image.data, rgb_image.data + (nx * ny * 3));

        mtmd_bitmap * bitmap_ptr = mtmd_bitmap_init(nx, ny, rgb_data.data());
        if (!bitmap_ptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create mtmd_bitmap from image");
            return "Failed to process image";
        }

        RCLCPP_INFO(this->get_logger(), "Image converted to mtmd_bitmap: %ux%u", nx, ny);

        std::string marker = mtmd_default_marker();

        std::string full_prompt =
            "<|im_start|>system\n"
            "你是一个有帮助的助手。你可以看到前方的景象。请用中文描述你看到的内容，并回答用户的问题。回答时使用第一人称，例如：'我看到了...'或'我前面有...'，不要使用'图片中'、'这张图'等词汇。<|im_end|>\n"
            "<|im_start|>user\n"
            "<" + std::string(marker) + ">\n" +
            prompt + "\n"
            "<|im_end|>\n"
            "<|im_start|>assistant\n";

        RCLCPP_INFO(this->get_logger(), "Full prompt with image marker");

        mtmd_input_text text;
        text.text = full_prompt.c_str();
        text.add_special = true;
        text.parse_special = true;

        const mtmd_bitmap * bitmaps[1] = { bitmap_ptr };

        mtmd_input_chunks * chunks_ptr = mtmd_input_chunks_init();
        if (!chunks_ptr) {
            RCLCPP_ERROR(this->get_logger(), "Failed to init input chunks");
            mtmd_bitmap_free(bitmap_ptr);
            return "Failed to initialize input";
        }

        int32_t res = mtmd_tokenize(mtmd_ctx_.get(), chunks_ptr, &text, bitmaps, 1);
        if (res != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to tokenize input: %d", res);
            mtmd_input_chunks_free(chunks_ptr);
            mtmd_bitmap_free(bitmap_ptr);
            return "Failed to tokenize input";
        }

        RCLCPP_INFO(this->get_logger(), "Tokenization successful");

        llama_pos n_past = 0;
        int32_t n_batch = 512;

        res = mtmd_helper_eval_chunks(
            mtmd_ctx_.get(),
            ctx_.get(),
            chunks_ptr,
            n_past,
            0,
            n_batch,
            true,
            &n_past
        );

        if (res != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to eval chunks: %d", res);
            mtmd_input_chunks_free(chunks_ptr);
            mtmd_bitmap_free(bitmap_ptr);
            return "Failed to evaluate chunks";
        }

        RCLCPP_INFO(this->get_logger(), "All chunks processed, n_past=%d, starting generation...", (int)n_past);

        std::string response = generate_response(n_past, goal_handle);

        mtmd_input_chunks_free(chunks_ptr);
        mtmd_bitmap_free(bitmap_ptr);

        return response;
    }

    std::string run_text_only_inference(const std::string& prompt, 
                                        const std::shared_ptr<GoalHandleLlama> goal_handle) {
        llama_memory_t mem = llama_get_memory(ctx_.get());
        llama_memory_seq_rm(mem, 0, -1, -1);
        
        std::string full_prompt =
            "<|im_start|>system\n"
            "你是一个有帮助的助手。请用中文友好地回复用户的消息。<|im_end|>\n"
            "<|im_start|>user\n"
            + prompt + "\n"
            "<|im_end|>\n"
            "<|im_start|>assistant\n";

        RCLCPP_INFO(this->get_logger(), "Full prompt: %s", full_prompt.c_str());

        std::vector<llama_token> prompt_tokens;
        prompt_tokens.resize(full_prompt.size() + 256);

        int32_t n_tokens = llama_tokenize(llama_model_get_vocab(model_.get()), full_prompt.c_str(), full_prompt.size(),
                                           prompt_tokens.data(), prompt_tokens.size(), true, true);
        if (n_tokens < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to tokenize prompt: %d", n_tokens);
            return "Failed to tokenize prompt";
        }

        prompt_tokens.resize(n_tokens);

        for (int i = 0; i < n_tokens; i++) {
            llama_batch batch = llama_batch_init(1, 0, 1);
            batch.n_tokens = 1;
            batch.token[0] = prompt_tokens[i];
            batch.pos[0] = i;
            batch.n_seq_id[0] = 1;
            batch.seq_id[0] = (llama_seq_id *)malloc(sizeof(llama_seq_id));
            batch.seq_id[0][0] = 0;
            batch.logits[0] = 1;

            if (llama_decode(ctx_.get(), batch)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode token %d", i);
                llama_batch_free(batch);
                return "Failed to decode prompt";
            }

            llama_batch_free(batch);
        }

        RCLCPP_INFO(this->get_logger(), "Prompt processed, n_past=%d, starting generation...", n_tokens);

        return generate_response(n_tokens, goal_handle);
    }

    std::string generate_response(llama_pos n_past, const std::shared_ptr<GoalHandleLlama> goal_handle) {
        auto sparams = llama_sampler_chain_default_params();
        llama_sampler * smpl = llama_sampler_chain_init(sparams);
        llama_sampler_chain_add(smpl, llama_sampler_init_top_k(50));
        llama_sampler_chain_add(smpl, llama_sampler_init_top_p(0.9, 1));
        llama_sampler_chain_add(smpl, llama_sampler_init_temp(0.7));
        llama_sampler_chain_add(smpl, llama_sampler_init_dist(0));

        std::string result_text;
        std::string sentence_buffer;
        llama_token new_token;
        int n_eval = 0;
        int max_tokens = 2048;
        bool eos_encountered = false;

        auto feedback = std::make_shared<LlamaAction::Feedback>();

        while (n_eval < max_tokens) {
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                llama_sampler_free(smpl);
                return result_text;
            }

            new_token = llama_sampler_sample(smpl, ctx_.get(), -1);

            if (llama_vocab_is_eog(llama_model_get_vocab(model_.get()), new_token)) {
                RCLCPP_INFO(this->get_logger(), "EOS token reached");
                eos_encountered = true;
                if (!sentence_buffer.empty()) {
                    publish_sentence(sentence_buffer);
                    sentence_buffer.clear();
                }
                break;
            }

            char buf[256];
            int n = llama_token_to_piece(llama_model_get_vocab(model_.get()), new_token, buf, sizeof(buf), 0, true);
            if (n > 0) {
                std::string token_str(buf, n);

                std::cout << token_str << std::flush;
                result_text += token_str;
                sentence_buffer += token_str;

                if (is_sentence_terminator(token_str)) {
                    RCLCPP_DEBUG(this->get_logger(), ">>> TERMINATOR DETECTED, publishing buffer: '%s'",
                        sentence_buffer.c_str());
                    publish_sentence(sentence_buffer);
                    sentence_buffer.clear();
                }

                feedback->progress = static_cast<float>(n_eval) / max_tokens;
                feedback->partial_response = result_text;
                feedback->status = "Generating...";
                goal_handle->publish_feedback(feedback);
            }

            llama_batch batch = llama_batch_init(1, 0, 1);
            batch.n_tokens = 1;
            batch.token[0] = new_token;
            batch.pos[0] = n_past;
            batch.n_seq_id[0] = 1;
            batch.seq_id[0] = (llama_seq_id *)malloc(sizeof(llama_seq_id));
            batch.seq_id[0][0] = 0;
            batch.logits[0] = 1;

            if (llama_decode(ctx_.get(), batch)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode");
                llama_batch_free(batch);
                break;
            }

            llama_batch_free(batch);

            n_past++;
            n_eval++;
        }

        if (!sentence_buffer.empty() && !eos_encountered) {
            publish_sentence(sentence_buffer);
            sentence_buffer.clear();
        }

        std::cout << std::endl;

        RCLCPP_INFO(this->get_logger(), "=== Full generated text ===");
        RCLCPP_INFO(this->get_logger(), "%s", result_text.c_str());
        RCLCPP_INFO(this->get_logger(), "=== End of generated text ===");

        llama_sampler_free(smpl);

        return result_text;
    }

    bool is_sentence_terminator(const std::string& str) {
        const std::vector<std::string> terminators = {",", ".", "?", "!", "。", "，", "？", "！"};
        for (const std::string& term : terminators) {
            if (str.find(term) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    void publish_sentence(const std::string& sentence) {
        std::string clean_sentence = sentence;
        clean_sentence.erase(std::remove(clean_sentence.begin(), clean_sentence.end(), '\n'), clean_sentence.end());
        clean_sentence.erase(std::remove(clean_sentence.begin(), clean_sentence.end(), '\r'), clean_sentence.end());

        if (clean_sentence.empty()) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing sentence: %s", clean_sentence.c_str());

        auto msg = std_msgs::msg::String();
        msg.data = clean_sentence;
        publisher_->publish(msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LlamaActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    return 0;
}
