#include <string>
#include <deque>
#include <nfd.h>
#include <chrono>

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
	UDP udp;
	std::shared_ptr<ZedTracker> zed_tracker;
	sl::float3 lastPosition;
	std::chrono::steady_clock::time_point t_start;
	vector<float> x_vel_history;
	vector<float> y_vel_history;
	vector<float> z_vel_history;
	vector<float> speed_history;

public:
	MainLayer(std::shared_ptr<ZedTracker> _zed_tracker) {
		udp.init();
		zed_tracker = _zed_tracker;
		lastPosition = sl::float3(0.0f, 0.0f, 0.0f);
		t_start = std::chrono::high_resolution_clock::now();

		x_vel_history.push_back(0.0f);
		y_vel_history.push_back(0.0f);
		z_vel_history.push_back(0.0f);
		speed_history.push_back(0.0f);

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

			// Fetch from ZED
			sl::float3 position = zed_tracker->fetch_track_pos();
			int clusterSize = zed_tracker->fetch_cluster_size();

			// Calculate delta time
			auto t_end = std::chrono::high_resolution_clock::now();
			double delta_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();
			t_start = t_end;

			// Calculate velocity (m/s)
			sl::float3 velocity = (position - lastPosition) * 10 / delta_time_ms;
			lastPosition = position;

			// Store velocity history
			x_vel_history.push_back(velocity.x);
			if (x_vel_history.size() > 100) x_vel_history.erase(x_vel_history.begin());
			y_vel_history.push_back(velocity.y);
			if (y_vel_history.size() > 100) y_vel_history.erase(y_vel_history.begin());
			z_vel_history.push_back(velocity.z);
			if (z_vel_history.size() > 100) z_vel_history.erase(z_vel_history.begin());
			speed_history.push_back(sl::float3::distance(sl::float3(0.0f, 0.0f, 0.0f), velocity));
			if (speed_history.size() > 100) speed_history.erase(speed_history.begin());
			
			// Send information over UDP
			char packet[64];
			snprintf(packet, 64, "%.4f %.4f %.4f %d ", position.x, position.y, position.z, clusterSize);
			udp.send(packet);
		}
	}

	virtual void OnUIRender() override {
		ImGui::Begin("Details");

		ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
		if (ImGui::BeginTabBar("MyTabBar", tab_bar_flags)) {
			if (ImGui::BeginTabItem("Network Config")) {

				// Network Settings
				ImGui::InputText("IP Address", udp.ip, 128);

				static char port[10];
				snprintf(port, 10, "%d", udp.port);
				ImGui::InputText("Port", port, 10);
				udp.port = atoi(port);

				if (ImGui::Button("Apply Network Config")) {
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

				static bool sdk_verbose = true;
				ImGui::Checkbox("SDK Verbose", &sdk_verbose);
				zed_tracker->config.init_parameters.sdk_verbose = (int)sdk_verbose;

				ImGui::Checkbox("Set Camera Static", &zed_tracker->config.positional_tracking_parameters.set_as_static);
				ImGui::Checkbox("Enable Tracking (across image flows)", &zed_tracker->config.obj_det_params.enable_tracking);
				ImGui::Checkbox("Enable Body Fitting (smooth skeleton moves)", &zed_tracker->config.obj_det_params.enable_body_fitting);

				static int detection_model_current_idx = (int)zed_tracker->config.obj_det_params.detection_model;
				char detection_model_options[512];
				enumToString<sl::DETECTION_MODEL>(detection_model_options);
				ImGui::Combo("Detection Model", &detection_model_current_idx, detection_model_options, 5);
				zed_tracker->config.obj_det_params.detection_model = (sl::DETECTION_MODEL)detection_model_current_idx;

				ImGui::InputFloat("Model Confidence", &zed_tracker->config.objectTracker_parameters_rt.detection_confidence_threshold, 1.0f, 10.0f, "%.3f");

				// Recording options
				ImGui::Checkbox("Record to SVO", &zed_tracker->config.enableRecording);
				static char svo_save_file_path[128] = "";
				ImGui::InputText("", svo_save_file_path, 128);
				ImGui::SameLine();
				if (ImGui::Button("Browse")) {
					nfdchar_t *outPath = NULL;
					nfdresult_t result = NFD_OpenDialog( NULL, NULL, &outPath );
							
					if ( result == NFD_OKAY ) {
						for (int i = 0; true; i++) {
							svo_save_file_path[i] = outPath[i];
							if (outPath[i] == '\0') {
								break;
							}
						}
						zed_tracker->config.recording_parameters.video_filename = sl::String(svo_save_file_path);
						NFD_Free(outPath);
					}
				}

				// Open and start ZED camera
				if (ImGui::Button("Open Camera")) {
					if (zed_tracker->isOpened()) {
						zed_tracker->close();
					}
				}

				ImGui::Dummy(ImVec2(0.0f, 20.0f));

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
			if (ImGui::BeginTabItem("Realtime Config")) {

				ImGui::InputFloat("Smoothing", &zed_tracker->tracking_options.smoothing, 0.1f, 1.0f, "%.3f");
				ImGui::InputFloat("Min Velocity", &zed_tracker->tracking_options.minVelocity, 0.1f, 1.0f, "%.3f");
				ImGui::InputFloat("Max Velocity", &zed_tracker->tracking_options.maxVelocity, 0.1f, 1.0f, "%.3f");
				ImGui::InputFloat("Cluster Threshold (cm)", &zed_tracker->tracking_options.clusterThreshold, 1.0f, 10.0f, "%.3f");
				ImGui::InputInt("Calibration Buffer", &zed_tracker->tracking_options.calibrationBuffer, 10, 100);

				if (ImGui::Button("Calibrate Camera")) {
					zed_tracker->calibrate();
				}
			
				ImGui::EndTabItem();
			}
			ImGui::EndTabBar();
		}

		ImGui::End();

		ImGui::Begin("Stats");

		char xVel[32];
		snprintf(xVel, 32, "X Vel: %.3f m/s", x_vel_history.back());
		ImGui::PlotLines("", x_vel_history.data(), x_vel_history.size(), 0, xVel, -5.0f, 5.0f, ImVec2(0, 80.0f));
		char yVel[32];
		snprintf(yVel, 32, "Y Vel: %.3f m/s", y_vel_history.back());
		ImGui::PlotLines("", y_vel_history.data(), y_vel_history.size(), 0, yVel, -5.0f, 5.0f, ImVec2(0, 80.0f));
		char zVel[32];
		snprintf(zVel, 32, "Z Vel: %.3f m/s", z_vel_history.back());
		ImGui::PlotLines("", z_vel_history.data(), z_vel_history.size(), 0, zVel, -5.0f, 5.0f, ImVec2(0, 80.0f));
		char speed[32];
		snprintf(speed, 32, "Speed: %.3f m/s", speed_history.back());
		ImGui::PlotLines("", speed_history.data(), speed_history.size(), 0, speed, -5.0f, 5.0f, ImVec2(0, 80.0f));

		ImGui::End();
	}
};