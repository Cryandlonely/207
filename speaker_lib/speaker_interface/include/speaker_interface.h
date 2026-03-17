#pragma once
/**
 * @file interface.h
 * @brief High-level speaker SDK interface for S600L / H600L.
 *
 * This is the ONLY file you need to read as a developer.
 * It wraps all SDK initialization, file transfer, playback, real-time voice,
 * device control, light, camera, and parameter APIs into simple method calls.
 *
 * All methods return data through output parameters or return values --
 * they do NOT print to stdout unless verbose mode is enabled.
 *
 * Quick start:
 *   SpeakerInterface speaker;
 *   speaker.set_verbose(true);  // optional: enable debug prints
 *   if (!speaker.init("192.168.144.67", 14556)) { ... handle error ... }
 *
 *   // Upload audio
 *   speaker.upload_audio("./local/test.wav");
 *
 *   // List files -- data is returned, not just printed
 *   SpeakerInterface::FileList files;
 *   if (speaker.list_audio(files)) {
 *       for (auto& f : files.files) { ... }
 *   }
 *
 *   // Get device info
 *   SpeakerInterface::DeviceInfo info;
 *   speaker.get_information(info);
 *
 *   // Playback
 *   speaker.play(0);
 *   speaker.set_volume(80);
 *
 *   // Parameters
 *   std::string value;
 *   speaker.get_param("SPK_LAN_IP", value);
 */

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

// Forward declarations -- developers do NOT need these headers.
namespace mavsdk { class Mavsdk; }

class SpeakerInterface {
public:
    SpeakerInterface();
    ~SpeakerInterface();

    // ==================== Data Structures ====================
    // These are plain structs with no SDK dependency.
    // Developers can use them freely without including any SDK header.

    /** @brief Result of a directory listing operation. */
    struct FileList {
        std::vector<std::string> dirs;   ///< Sub-directory names
        std::vector<std::string> files;  ///< File names
    };

    /** @brief Speaker hardware/firmware information. */
    struct DeviceInfo {
        std::string vendor_name;
        std::string model_name;
        std::string serial_number;
        std::string firmware_version;
    };

    /** @brief Storage capacity information. */
    struct StorageInfo {
        float total_mib{0.0f};      ///< Total storage in MiB
        float used_mib{0.0f};       ///< Used storage in MiB
        float available_mib{0.0f};  ///< Available storage in MiB
    };

    /** @brief Audio file metadata. */
    struct AudioFileInfo {
        std::string name;
        uint32_t index{0};
        std::string format;
        float length_seconds{0.0f};
    };

    /** @brief A single named parameter (int, float, or string). */
    struct ParamValue {
        std::string name;
        enum class Type { Int, Float, String } type{Type::String};
        int int_val{0};
        float float_val{0.0f};
        std::string string_val;
    };

    // ==================== Configuration ====================

    /**
     * @brief Enable or disable verbose (debug) output to stderr.
     * @param enabled  true = print progress/debug messages, false = silent (default).
     *
     * When disabled, all methods are completely silent and only return data.
     * The demo CLI sets this to true for interactive use.
     */
    void set_verbose(bool enabled);

    /** @brief Check whether verbose mode is on. */
    bool is_verbose() const;

    // ==================== Initialization ====================

    /**
     * @brief Initialize the SDK and connect to the speaker over TCP.
     * @param ip   Device IP address, e.g. "192.168.144.67"
     * @param port MAVSDK TCP port, e.g. 14556
     * @return true on success (system discovered), false on failure.
     *
     * This performs the full lifecycle:
     *   1. Create MAVSDK instance with GroundStation role.
     *   2. Connect via TCP (tcpout://<ip>:<port>).
     *   3. Wait up to 10 s for speaker system discovery.
     *   4. Create internal FTP-pro backend (port + 10).
     *   5. Set up status/log subscriptions.
     */
    bool init(const std::string& ip, int port);

    /** @brief Check whether init() succeeded and the system is connected. */
    bool is_connected() const;

    // ==================== Audio File Operations ====================

    /**
     * @brief Upload a local audio file into the speaker's media/ directory.
     * @param local_path Full local path, e.g. "./audio/test.wav"
     * @return true on success.
     *
     * The remote destination is always media/<basename>.
     */
    bool upload_audio(const std::string& local_path);

    /**
     * @brief List audio files in the speaker's media/ directory.
     * @param[out] out  Populated with directory entries on success.
     * @return true on success.
     */
    bool list_audio(FileList& out);

    /**
     * @brief Rename an audio file on the speaker.
     * @param old_name Current filename (relative to media/).
     * @param new_name New filename.
     * @return true on success.
     */
    bool rename_audio(const std::string& old_name, const std::string& new_name);

    /**
     * @brief Delete an audio file from the speaker.
     * @param filename Filename to delete (relative to media/).
     * @return true on success.
     */
    bool delete_audio(const std::string& filename);

    /**
     * @brief Subscribe to audio info updates for a given file.
     * @param filename The audio file name to query.
     * @param callback Called each time audio info is received.
     *
     * The subscription stays active until the SpeakerInterface is destroyed.
     */
    void subscribe_audio_info(const std::string& filename,
                              std::function<void(const AudioFileInfo&)> callback);

    // ==================== Firmware Update ====================

    /**
     * @brief Upload firmware and restart the speaker.
     * @param firmware_local_path Local path to the firmware file.
     * @return true if upload succeeded and restart was issued.
     *
     * The firmware is uploaded to update/<basename>, then restart() is called.
     */
    bool upload_firmware(const std::string& firmware_local_path);

    // ==================== Playback Control ====================

    /**
     * @brief Play an audio file once by index from the subscribe_audio_info
     * @param index 0-based index in the media list.
     * @return true on success.
     */
    bool play(int index);

    /**
     * @brief Play an audio file in loop mode by index.
     * @param index 0-based index in the media list.
     * @return true on success.
     */
    bool loop_play(int index);

    /** @brief Stop the current playback. @return true on success. */
    bool stop();

    // ==================== Real-Time Voice ====================

    /**
     * @brief Start real-time listening (speaker captures audio and streams to GCS).
     *
     * Before calling this, ensure SPK_LAN_IP param is set to the GCS IP
     * so the speaker knows where to push the audio stream.
     * @return true on success.
     */
    bool start_listen();

    /** @brief Stop real-time listening. @return true on success. */
    bool stop_listen();

    /**
     * @brief Start real-time speaking (GCS pushes audio stream to speaker).
     *
     * Before calling this, start your audio stream sender (e.g. ffmpeg)
     * targeting <speaker_ip>:15557.
     * @return true on success.
     */
    bool start_speak();

    /** @brief Stop real-time speaking. @return true on success. */
    bool stop_speak();

    // ==================== Volume & Angle ====================

    /**
     * @brief Set speaker volume.
     * @param volume_percent 0.0 ~ 100.0
     * @return true on success.
     */
    bool set_volume(float volume_percent);

    /**
     * @brief Set speaker gimbal angle (H600L only).
     * @return true on success.
     */
    bool set_angle(float roll, float pitch, float yaw);

    // ==================== Light Control ====================

    /**
     * @brief Set the built-in light mode.
     * @param mode Light mode index from 0 to 4.
     * @return true on success.
     */
    bool set_light_mode(int mode);

    /**
     * @brief Turn the built-in light on or off.
     * @param on true = on, false = off.
     * @return true on success.
     */
    bool set_light_on_off(bool on);

    /**
     * @brief Turn the external light on or off (S600L only).
     * @param on true = on, false = off.
     * @return true on success.
     */
    bool set_external_light_on_off(bool on);

    // ==================== Camera (S600L only) ====================

    /** @brief Take a photo. @return true on success. */
    bool take_picture();

    /** @brief Start video recording. @return true on success. */
    bool start_video();

    /** @brief Stop video recording. @return true on success. */
    bool stop_video();

    /**
     * @brief Switch camera lens.
     * @param lens_id 0 or 1.
     * @return true on success.
     */
    bool switch_lens(int lens_id);

    /** @brief Camera network parameters structure */
    struct CameraNetworkParams {
        std::string ip_address;  // e.g., "192.168.144.67"
        int bitrate;             // 0=Default, 1=0.5M, 2=1M, 3=2M, 4=3M, 5=4M, 6=0.1M, 7=0.2M
        int resolution;          // 0=Default, 1=1080p, 2=720p, 3=576p
        int encoding;            // 0=Default, 1=H264, 2=H265
    };

    /**
     * @brief Get camera network parameters.
     * @param[out] out Populated with network params on success.
     * @return true on success.
     */
    bool get_camera_network_params(CameraNetworkParams& out);

    /**
     * @brief Set camera network parameters.
     * @param params Network parameters to set.
     * @return true on success.
     */
    bool set_camera_network_params(const CameraNetworkParams& params);

    // ==================== Device Info & Parameters ====================

    /**
     * @brief Get speaker hardware/firmware information.
     * @param[out] out  Populated with device info on success.
     * @return true if info was retrieved.
     *
     * Note: this subscribes to a one-shot information update and blocks briefly.
     */
    bool get_information(DeviceInfo& out);

    /**
     * @brief Get storage capacity information.
     * @param[out] out  Populated with storage info on success.
     * @return true if info was retrieved.
     */
    bool get_storage(StorageInfo& out);

    /** @brief Restart the speaker. @return true on success. */
    bool restart();

    /**
     * @brief Set a device parameter.
     * @param name  Parameter name, e.g. "SPK_LAN_IP".
     * @param value Parameter value as string. Numeric strings are auto-detected
     *              and sent as int/float; otherwise sent as custom string param.
     * @return true on success.
     *
     * Example: set_param("SPK_LAN_IP", "192.168.144.20")
     */
    bool set_param(const std::string& name, const std::string& value);

    /**
     * @brief Get a device parameter value.
     * @param name       Parameter name.
     * @param[out] out   Populated with the parameter value on success.
     * @return true if the parameter was found.
     */
    bool get_param(const std::string& name, ParamValue& out);

    /**
     * @brief Get all device parameters.
     * @param[out] out   Populated with all parameters on success.
     * @return true on success (even if the list is empty).
     */
    bool get_all_params(std::vector<ParamValue>& out);

private:
    struct Impl;
    std::unique_ptr<Impl> _impl;
};
