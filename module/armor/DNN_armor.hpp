#pragma once

#include <fmt/color.h>
#include <fmt/core.h>

#include <string>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <onnxruntime/onnxruntime_cxx_api.h>
#include <onnxruntime/onnxruntime_c_api.h>

namespace DNN_armor
{
    auto idntifier = fmt::format(fg(fmt::color::green) | fmt::emphasis::bold, "DNN_INFO");
    struct DNN_Data
    {
        int debug_mode;
        int height;
        int width;
    };
    struct Object
    {
        int x1, y1, x2, y2, label, prob;
    };
    class DNN_Model;
    class DNN_Dectect
    {
    public:
        DNN_Dectect(std::string input_path);
        void Detect(cv::Mat &src_img, DNN_Model &model);
        Ort::Value prepare_input(cv::Mat &bgr_image);
        cv::Mat prepareInput(cv::Mat &src_img, int width, int height);
        float iou(std::vector<float> &rec_1, std::vector<float> &rec_2);

    private:
        DNN_Data DNN_Config_;
        cv::Mat img;
        std::vector<Ort::Value> output;
    };
    class DNN_Model
    {
    public:
        DNN_Model(std::string input_path, Ort::Env &env, Ort::SessionOptions &session_options) : session(env, input_path.c_str(), session_options)
        {
            model_path = input_path;
            fmt::print("[{}] Load Success, the model is: {}\n", idntifier, model_path);
            // Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
            printModelInfo(session, allocator);
        }
        inline void load(std::string onnx_model_path);
        void printModelInfo(Ort::Session &session, Ort::AllocatorWithDefaultOptions &allocator);

        Ort::Session session;
        std::string model_path;
        Ort::AllocatorWithDefaultOptions allocator;
    };

    class DNN_Model_CV
    {
    public:
        DNN_Model_CV(std::string input_path);
        void Init(std::string onnx_model_path);
        struct DNN_Data
        {
            int debug_mode;
            int height;
            int width;
            float mean_vals[3];
            float norm_vals[3];
        };

        DNN_Data DNN_Config_;
        cv::dnn::Net net;

    private:
        std::string onnx_path;
    };

    class DNN_Detect_CV
    {
    public:
        DNN_Detect_CV();
        void Detect(cv::Mat &src_img, DNN_Model_CV &model);
        cv::Mat imgResize(cv::Mat &src_img, DNN_Model_CV &model);


        

        inline float min(float x, float y)
        {
            return (x < y) ? x : y;
        }

        struct Object
        {
            cv::Rect_<float> rect;
            int label;
            float prob;
        };


    private:
        int num_grid;
        int num_class;
        void generate_yolox_proposals(const float *feat_ptr, std::vector<Object> &objects);
    };
}