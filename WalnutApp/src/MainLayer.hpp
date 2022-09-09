#include <string>
#include <deque>
#include <nfd.h>
#include <chrono>
#include <math.h>

#include "Walnut/Application.h"
#include "magic_enum.hpp"
#include "ZedTracker.hpp"
#include "UDP.hpp"

template<typename T>
void enumToString(char *retval) {
	constexpr auto enumNames = magic_enum::enum_names<T>();
	int n = 0;
	for (int i = 0; i < enumNames.size(); i++) {
		for (int j = 0; j < enumNames[i].size(); j++) {
			retval[n++] = enumNames[i][j];
		}
		retval[n++] = '\0';
	}
	retval[n++] = '\0';
}

class MainLayer : public Walnut::Layer {
private:
	UDP livelink;
	UDP udp;
	std::shared_ptr<ZedTracker> zed_tracker;
	std::shared_ptr<Walnut::Image> image_preview;
	sl::float3 lastPosition;
	std::chrono::steady_clock::time_point t_start;
	vector<vector<float>> graphs;
	float armTrigger[2];

public:
	MainLayer(std::shared_ptr<ZedTracker> _zed_tracker) {
		livelink.setDest("10.104.10.219", 54321);
		livelink.init();
		udp.setDest("10.104.10.219", 9321);
		udp.init();
		zed_tracker = _zed_tracker;
		lastPosition = sl::float3(0.0f, 0.0f, 0.0f);
		t_start = std::chrono::high_resolution_clock::now();

		// Camera options
		zed_tracker->config.init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
		zed_tracker->config.init_parameters.depth_mode = sl::DEPTH_MODE::NEURAL;
		zed_tracker->config.init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP; // Unreal Engine
		zed_tracker->config.init_parameters.sdk_verbose = 1;
		zed_tracker->config.positional_tracking_parameters.set_as_static = true;
		zed_tracker->config.obj_det_params.enable_tracking = true; // track people across images flow
		zed_tracker->config.obj_det_params.enable_body_fitting = false; // smooth skeletons moves
		zed_tracker->config.obj_det_params.body_format = sl::BODY_FORMAT::POSE_18;
		zed_tracker->config.obj_det_params.detection_model = sl::DETECTION_MODEL::HUMAN_BODY_ACCURATE;
		zed_tracker->config.objectTracker_parameters_rt.detection_confidence_threshold = 25.0f;
		zed_tracker->config.enableRecording = false;
		zed_tracker->config.recording_parameters.compression_mode = sl::SVO_COMPRESSION_MODE::H264;
		zed_tracker->config.recording_parameters.video_filename = sl::String("./out.svo");
		// Tracking options
		zed_tracker->tracking_options.calibrationBuffer = 1000;
		zed_tracker->tracking_options.minVelocity = 1.3f;
		zed_tracker->tracking_options.maxVelocity = 20.0f;
		zed_tracker->tracking_options.smoothing = 0.4f;
		zed_tracker->tracking_options.clusterThreshold = 200.0f;
	}

	virtual void OnAttach() override {
	}

	virtual void OnDetach() override {
		udp.destroy();
		zed_tracker->destroy();
	}

	virtual void OnUpdate(float ts) override {
		if (zed_tracker->isOpened()) {
			zed_tracker->fetch();

			// Get variables
			ZedRuntimeData runData = zed_tracker->runtime_data;

			// Fetch from ZED
			sl::float3 position = zed_tracker->fetch_track_pos();
			int clusterSize = zed_tracker->fetch_cluster_size();

			// Get image preview
			cv::Mat image_mat = zed_tracker->fetch_image(true);
			cv::cvtColor(image_mat, image_mat, cv::COLOR_BGRA2RGBA);
			sl::Resolution imgRes = zed_tracker->config.camera_config.resolution;
			if (image_preview.use_count() == 0) {
				image_preview = std::make_shared<Walnut::Image>(imgRes.width, imgRes.height, Walnut::ImageFormat::RGBA, image_mat.data);
			}
			else if (imgRes.width != image_preview->GetWidth() || imgRes.height != image_preview->GetHeight()) {
				image_preview->Resize(imgRes.width, imgRes.height);
			}
			else {
				image_preview->SetData(image_mat.data);
			}

			// Calculate delta time
			auto t_end = std::chrono::high_resolution_clock::now();
			double delta_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
			t_start = t_end;

			// Calculate velocity (m/s)
			sl::float3 velocity = (position - lastPosition) * 10 / delta_time_ms;
			lastPosition = position;

			// Calculate arm controls
			Gesture armGestures[2][2] = {
				{Gesture::LeftArmStraight, Gesture::LeftArmControl},
				{Gesture::RightArmStraight, Gesture::RightArmControl}
			};
			bool armControl[2];
			float armRange[2] = {0.0f, 0.0f};
			for (int i = 0; i < 2; i++) {
				armTrigger[i] = runData.headTrackId != -1 ? runData.gestureCompletion[runData.headTrackId][(int)armGestures[i][0]] : 0.0f;
				armControl[i] = runData.headTrackId != -1 ? runData.armControl[runData.headTrackId][i] : false;
				for (GestureDetection detection : runData.gestures[runData.headTrackId]) {
					if (detection.gesture == armGestures[i][1] && armControl[i]) {
						armRange[i] = detection.percentComplete;
					}
				}
			}

			// Store velocity history
			float graphValues[] = {
				armRange[0],
				armRange[1],
				1000.0 / delta_time_ms,
				velocity.x,
				velocity.y,
				velocity.z,
				sl::float3::distance(sl::float3(0.0f, 0.0f, 0.0f), velocity)
			};
			int numValues = sizeof(graphValues) / sizeof(float);
			int numToAdd = numValues - graphs.size();
			for (int i = 0; i < numToAdd; i++) {
				graphs.push_back(vector<float>(100, 0.0f));
			}
			for (int i = 0; i < graphs.size(); i++) {
				graphs[i].push_back(graphValues[i]);
				if (graphs[i].size() > 100) graphs[i].erase(graphs[i].begin());
			}
			
			// Send information over UDP
			char packet[64];
			snprintf(packet, 64, "%.4f %.4f %.4f 0.0 0.0 0.0 0.0 ", position.x, position.y, position.z);
			livelink.send(packet);
			snprintf(packet, 64, "%d %.4f %.4f %.4f %.4f ", clusterSize, armRange[0], armRange[1], armTrigger[0], armTrigger[1]);
			udp.send(packet);
		}
	}

	virtual void OnUIRender() override {
		ImGui::Begin("Details");

		ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
		if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags)) {
			if (ImGui::BeginTabItem("Network Config")) {

				// Network Settings
				static char ipBuff[32];
				snprintf(ipBuff, 32, "%s", udp.ip);
				ImGui::InputText("IP Address", ipBuff, 128);

				static char portBuff[10];
				snprintf(portBuff, 10, "%d", livelink.port);
				ImGui::InputText("LiveLink Port", portBuff, 10);
				livelink.setDest(ipBuff, atoi(portBuff));

				snprintf(portBuff, 10, "%d", udp.port);
				ImGui::InputText("Port", portBuff, 10);
				udp.setDest(ipBuff, atoi(portBuff));

				if (ImGui::Button("Apply Network Config")) {
					livelink.init();
					udp.init();
				}
				
				ImGui::EndTabItem();
			}
			if (ImGui::BeginTabItem("Camera Config")) {

				// Camera Settings
				static int camera_resolution_current_idx = (int)sl::RESOLUTION::HD1080;
				char resolution_options[512];
				enumToString<sl::RESOLUTION>(resolution_options);
				ImGui::Combo("Resolution", &camera_resolution_current_idx, resolution_options, 5);
				zed_tracker->config.init_parameters.camera_resolution = (sl::RESOLUTION)camera_resolution_current_idx;

				static int depth_mode_current_idx = (int)sl::DEPTH_MODE::NEURAL;
				char depth_mode_options[512];
				enumToString<sl::DEPTH_MODE>(depth_mode_options);
				ImGui::Combo("Depth Mode", &depth_mode_current_idx, depth_mode_options, 5);
				zed_tracker->config.init_parameters.depth_mode = (sl::DEPTH_MODE)depth_mode_current_idx;

				static int coordinate_system_current_idx = (int)sl::COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
				char coordinate_system_options[512];
				enumToString<sl::COORDINATE_SYSTEM>(coordinate_system_options);
				ImGui::Combo("Coordinate System", &coordinate_system_current_idx, coordinate_system_options, 5);
				zed_tracker->config.init_parameters.coordinate_system = (sl::COORDINATE_SYSTEM)coordinate_system_current_idx;

				static int detection_model_current_idx = (int)zed_tracker->config.obj_det_params.detection_model;
				char detection_model_options[512];
				enumToString<sl::DETECTION_MODEL>(detection_model_options);
				ImGui::Combo("Detection Model", &detection_model_current_idx, detection_model_options, 5);
				zed_tracker->config.obj_det_params.detection_model = (sl::DETECTION_MODEL)detection_model_current_idx;

				ImGui::InputFloat("Model Confidence", &zed_tracker->config.objectTracker_parameters_rt.detection_confidence_threshold, 1.0f, 10.0f, "%.3f");

				// Recording options
				ImGui::Checkbox("Record to SVO", &zed_tracker->config.enableRecording);
				if (zed_tracker->config.enableRecording) {
					static char svo_save_file_path[128] = "";
					ImGui::InputText("", svo_save_file_path, 128);
					ImGui::SameLine();
					if (ImGui::Button("Browse")) {
						nfdchar_t *saveOutPath = NULL;
						nfdresult_t result = NFD_OpenDialog( NULL, NULL, &saveOutPath );
								
						if ( result == NFD_OKAY ) {
							for (int i = 0; true; i++) {
								svo_save_file_path[i] = saveOutPath[i];
								if (saveOutPath[i] == '\0') {
									break;
								}
							}
							NFD_Free(saveOutPath);
						}
					}
					zed_tracker->config.recording_parameters.video_filename = sl::String(svo_save_file_path);
				}

				// Open and start ZED camera
				if (ImGui::Button("Open Camera", ImVec2(200.0f, 50.0f))) {
					if (zed_tracker->isOpened()) {
						zed_tracker->close();
					}
					zed_tracker->config.init_parameters.input = sl::InputType();
					zed_tracker->configureCamera();
				}
				ImGui::SameLine();
				// Close Zed camera
				if (ImGui::Button("Close Camera", ImVec2(200.0f, 50.0f))) {
					if (zed_tracker->isOpened()) {
						zed_tracker->close();
					}
					zed_tracker->close();
				}

				ImGui::Dummy(ImVec2(0.0f, 20.0f));

				if (!zed_tracker->config.enableRecording) {
					static char svo_file_path[128] = "C:\\Users\\patricks\\Downloads\\out.svo";
					ImGui::InputText("", svo_file_path, 128);
					ImGui::SameLine();
					if (ImGui::Button("Browse")) {
						nfdchar_t *outPath = NULL;
						nfdresult_t result = NFD_OpenDialog( NULL, NULL, &outPath );
								
						if ( result == NFD_OKAY ) {
							for (int i = 0; true; i++) {
								svo_file_path[i] = outPath[i];
								if (outPath[i] == '\0') {
									break;
								}
							}
							NFD_Free(outPath);
						}
					}
					ImGui::SameLine();
					if (ImGui::Button("Start Playback")) {
						if (zed_tracker->isOpened()) {
							zed_tracker->close();
						}
						zed_tracker->configurePlayback(svo_file_path);
					}
				
					ImGui::EndTabItem();
				}
			}
			if (ImGui::BeginTabItem("Realtime Config")) {

				if (zed_tracker->isOpened()) {
					ImGui::InputFloat("Smoothing", &zed_tracker->tracking_options.smoothing, 0.1f, 1.0f, "%.3f");
					ImGui::InputFloat("Min Velocity", &zed_tracker->tracking_options.minVelocity, 0.1f, 1.0f, "%.3f");
					ImGui::InputFloat("Max Velocity", &zed_tracker->tracking_options.maxVelocity, 0.1f, 1.0f, "%.3f");
					ImGui::InputFloat("Cluster Threshold (cm)", &zed_tracker->tracking_options.clusterThreshold, 1.0f, 10.0f, "%.3f");
					ImGui::InputInt("Calibration Buffer", &zed_tracker->tracking_options.calibrationBuffer, 10, 100);

					// Camera settings
					static int exposure = zed_tracker->zed.getCameraSettings(VIDEO_SETTINGS::EXPOSURE);
					ImGui::SliderInt("Camera Exposure", &exposure, 0, 100);
					zed_tracker->zed.setCameraSettings(VIDEO_SETTINGS::EXPOSURE, exposure);
					static int gain = zed_tracker->zed.getCameraSettings(VIDEO_SETTINGS::GAIN);
					ImGui::SliderInt("Camera Gain", &gain, 0, 100);
					zed_tracker->zed.setCameraSettings(VIDEO_SETTINGS::GAIN, gain);
					static int gamma = zed_tracker->zed.getCameraSettings(VIDEO_SETTINGS::GAMMA);
					ImGui::SliderInt("Camera Gamma", &gamma, 1, 9);
					zed_tracker->zed.setCameraSettings(VIDEO_SETTINGS::GAMMA, gamma);
					static int sharpness = zed_tracker->zed.getCameraSettings(VIDEO_SETTINGS::SHARPNESS);
					ImGui::SliderInt("Camera Sharpness", &sharpness, 0, 8);
					zed_tracker->zed.setCameraSettings(VIDEO_SETTINGS::SHARPNESS, sharpness);

					if (ImGui::Button("Calibrate Camera")) {
						zed_tracker->calibrate();
					}
				}
				else {
					ImGui::Text("Camera not running!");
				}
			
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
		}

		ImGui::End();

		ImGui::Begin("Stats");

		ImGui::ProgressBar(armTrigger[0], ImVec2(0, 80.0f), "Left Trigger");
		ImGui::ProgressBar(armTrigger[1], ImVec2(0, 80.0f), "Right Trigger");

		char labelBuff[64];
		char labels[][64] = {
			"Left Arm Control: %.2f",
			"Right Arm Control: %.2f",
			"%.2f fps",
			"X Vel: %.3f m/s",
			"Y Vel: %.3f m/s",
			"Z Vel: %.3f m/s",
			"Speed: %.3f m/s"
		};
		float ranges[][2] = {
			{ 0.0f, 1.0f },
			{ 0.0f, 1.0f },
			{ 0.0f, 30.0f },
			{ -5.0f, 5.0f },
			{ -5.0f, 5.0f },
			{ -5.0f, 5.0f },
			{ -5.0f, 5.0f }
		};

		for (int i = 0; i < graphs.size(); i++) {
			snprintf(labelBuff, 64, labels[i], graphs[i].back());
			ImGui::PlotLines("", graphs[i].data(), graphs[i].size(), 0, labelBuff, ranges[i][0], ranges[i][1], ImVec2(0, 80.0f));
		}

		ImGui::End();

		ImGui::Begin("Gestures");
		
		constexpr auto gestureNames = magic_enum::enum_names<Gesture>();
		
		char buff[512];
		for (std::pair<int, std::vector<GestureDetection>> personPair : zed_tracker->runtime_data.gestures) {
			int personId = personPair.first;
			for (GestureDetection detection : personPair.second) {
				snprintf(buff, 512, "Person Id: %d\nGesture: %s\nPercent Complete: %.3f\nDescription: %s\n", personId, gestureNames.at((int)detection.gesture).data(), detection.percentComplete, detection.description.c_str());
				ImGui::Text(buff);
			}
		}

		ImGui::End();

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