#include "Walnut/Application.h"
#include "Walnut/Image.h"
#include "ZedTracker.hpp"

class ZedCameraPreviewLayer : public Walnut::Layer {
private:
	std::shared_ptr<ZedTracker> zed_tracker;
	std::shared_ptr<Walnut::Image> image_preview;

public:
	ZedCameraPreviewLayer(std::shared_ptr<ZedTracker> _zed_tracker) {
		zed_tracker = _zed_tracker;
	}

	virtual void OnAttach() override {
	}

	virtual void OnDetach() override {
		zed_tracker->destroy();
	}

	virtual void OnUpdate(float ts) override {
    if (zed_tracker->isOpened()) {
      // Get image preview
      cv::Mat image_mat = zed_tracker->fetch_image();
      cv::cvtColor(image_mat, image_mat, cv::COLOR_BGRA2RGBA);
      sl::Resolution imgRes = zed_tracker->config.camera_config.resolution;
			if (image_preview.use_count() == 0) {
				image_preview = std::make_shared<Walnut::Image>(imgRes.width, imgRes.height, Walnut::ImageFormat::RGBA, image_mat.data);
			}
			else {
				image_preview->SetData(image_mat.data);
			}
		}
	}

	virtual void OnUIRender() override {
		ImGui::Begin("ZED Camera Preview");

    if (image_preview != nullptr) {
			// Resize to window
			uint32_t width = ImGui::GetWindowSize().x;
			uint32_t height = width * (double)image_preview->GetHeight() / image_preview->GetWidth();
      ImGui::Image(image_preview->GetDescriptorSet(), ImVec2(width, height));
    }

		ImGui::End();
	}
};