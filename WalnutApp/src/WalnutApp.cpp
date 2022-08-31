#include "Walnut/Application.h"
#include "Walnut/Image.h"
#include "ZedTracker.hpp"
#include "imageutils.h"

class ConfigLayer : public Walnut::Layer {
private:
	std::shared_ptr<ZedTracker> zed_tracker;

public:
	ConfigLayer(std::shared_ptr<ZedTracker> _zed_tracker) {
		zed_tracker = _zed_tracker;

		// Camera options
		zed_tracker->config.init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
		zed_tracker->config.init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;
		// zed_tracker->config.init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // Default
		zed_tracker->config.init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP; // Unreal Engine
		zed_tracker->config.init_parameters.sdk_verbose = 1;
		zed_tracker->config.positional_tracking_parameters.set_as_static = true;
		zed_tracker->config.obj_det_params.enable_tracking = true; // track people across images flow
		zed_tracker->config.obj_det_params.enable_body_fitting = false; // smooth skeletons moves
		zed_tracker->config.obj_det_params.body_format = sl::BODY_FORMAT::POSE_18;
		zed_tracker->config.obj_det_params.detection_model = sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE;
		zed_tracker->config.objectTracker_parameters_rt.detection_confidence_threshold = 25.0f;
		zed_tracker->config.recordingParameters.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
		zed_tracker->config.recordingParameters.video_filename = sl::String("./out.svo");
		// Tracking options
		zed_tracker->tracking_options.calibrationBuffer = 1000;
		zed_tracker->tracking_options.minVelocity = 1.3f;
		zed_tracker->tracking_options.maxVelocity = 20.0f;
		zed_tracker->tracking_options.smoothing = 0.4f;
		zed_tracker->tracking_options.clusterThreshold = 2000.0f;
	}

	virtual void OnAttach() override {

	}

	virtual void OnDetach() override {
		zed_tracker->destroy();
	}

	virtual void OnUpdate(float ts) override {

	}

	virtual void OnUIRender() override {
		ImGui::Begin("Hello");
		char buffer[50];
		snprintf(buffer, 50, "Smoothing: %.4f", zed_tracker->tracking_options.maxVelocity);
		ImGui::Text(buffer);

		if (ImGui::Button("Configure Camera")) {
			try {
				zed_tracker->configureCamera();
			}
			catch(const std::exception& e) {
				std::cerr << e.what() << std::endl;
				ImGui::TextColored(ImVec4(0.8, 0, 0, 1), "Error configuring camera!");
			}
		}

		if (ImGui::Button("Calibrate Camera")) {
			zed_tracker->calibrate();
		}

		if (zed_tracker->fetch()) {
			cv::Mat image_mat = zed_tracker->fetch_image();
			OpenCVImage img;
			img.LoadCVMat(image_mat);
			sl::Resolution imgRes = zed_tracker->config.camera_config.resolution;
			ImGui::Image(img.getTexture(), ImVec2(imgRes.width, imgRes.height));
		}

		ImGui::End();

		ImGui::ShowDemoWindow();
	}
};

Walnut::Application* Walnut::CreateApplication(int argc, char** argv) {
	Walnut::ApplicationSpecification spec;
	spec.Name = "Zed Tracker";

	// Create ZED tracker
	std::shared_ptr<ZedTracker> zed_tracker = std::make_shared<ZedTracker>();

	// Layers
	std::shared_ptr<ConfigLayer> configLayer = std::make_shared<ConfigLayer>(zed_tracker);

	Walnut::Application* app = new Walnut::Application(spec);
	app->PushLayer(configLayer);
	app->SetMenubarCallback([app]() {
		if (ImGui::BeginMenu("File")) {
			if (ImGui::MenuItem("Exit")) {
				app->Close();
			}
			ImGui::EndMenu();
		}
	});
	return app;
}