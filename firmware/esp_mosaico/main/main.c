#include <stdio.h>
#include <inttypes.h>
#include "esp_chip_info.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tinyusb_default_config.h"
#include "tinyusb_cdc_acm.h"
#include "tinyusb_console.h"
#include "soc/lp_system_reg.h"
#include "oft_board.h"
#include "oft_ble.h"
#include "oft_network.h"
#include "oft_udp.h"
#include "oft_rx_queue.h"
#include "oft_motion.h"
#include "oft_ui.h"
#include "oft_touch.h"
#include "oft_peripherals.h"
#include "oft_gauge.h"
#include "oft_video.h"
#include "esp_heap_caps.h"
#include "tusb.h"
#include "unity.h"
#include <string.h>
#include <unistd.h>

void setUp(void) {}
void tearDown(void) {}
void oft_protocol_run_selftests(void);
void __wrap_unity_putc(int c) { putchar(c); }
void __wrap_unity_flush(void) { fflush(stdout); }

static volatile bool download_requested;
static unsigned download_stage;
static int64_t download_deadline;
static volatile bool early_boot=true;
static void early_download_watch(void *unused)
{
    (void)unused;
    while(early_boot){
        // No radios have started in this phase. Preserve a recovery path even
        // when a future display constructor or board selftest aborts startup.
        if(download_requested){REG_SET_BIT(LP_SYSTEM_REG_SYS_CTRL_REG,LP_SYSTEM_REG_FORCE_DOWNLOAD_BOOT);esp_restart();}
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}
static void coding_changed(int itf, cdcacm_event_t *event)
{
    (void)itf;
    /* Explicit 1200-baud touch requests ROM download; this is a volatile
       ESP32-S31 register, not an eFuse or boot-image modification. */
    if (event->line_coding_changed_data.p_line_coding->bit_rate == 1200) {
        download_requested = true;
    }
}

/* Tests gate radio startup; live control also requires profile, session,
   heartbeat, priming-center receipt and a fresh physical touch gesture. */
void app_main(void)
{
    const tinyusb_config_t usb = TINYUSB_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(tinyusb_driver_install(&usb));
    const tinyusb_config_cdcacm_t cdc = {.callback_line_coding_changed = coding_changed};
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&cdc));
    ESP_ERROR_CHECK(tinyusb_console_init(TINYUSB_CDC_ACM_0));
    if(xTaskCreatePinnedToCore(early_download_watch,"oft_early_usb",2048,NULL,6,NULL,0)!=pdPASS)ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    vTaskDelay(pdMS_TO_TICKS(3000)); // Enumerate CDC before display/tests; explicit1200-baud recovery stays available.
    esp_err_t board_result=oft_board_display_start();
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    oft_protocol_run_selftests();
    bool protocol_tests_ran=true;
    esp_err_t radio_result=ESP_ERR_INVALID_STATE;
    early_boot=false;
#ifdef OFT_CABAC32
    const unsigned required_tests=79;
#else
    const unsigned required_tests=78;
#endif
    if(board_result==ESP_OK&&Unity.NumberOfTests>=required_tests&&!Unity.TestFailures&&!Unity.TestIgnores){
        esp_err_t motion_result=oft_motion_start();
        printf("OFT_MOTION start=%s\n",esp_err_to_name(motion_result));
        printf("OFT_PERIPHERALS start=%s\n",esp_err_to_name(oft_peripherals_start()));
        radio_result=oft_network_start();
        if(radio_result==ESP_OK)radio_result=oft_ble_start();
        if(radio_result==ESP_OK)radio_result=oft_udp_start();
        if(radio_result==ESP_OK)oft_video_start();
    }
    static char command[64];size_t command_size=0;bool discard_command=false;
    while (true) {
        if (download_requested) {
            oft_motion_haptic(false);
            if(!download_stage){oft_udp_stop();download_stage=1;download_deadline=esp_timer_get_time()+5000000;}
            oft_udp_snapshot_t u;oft_udp_snapshot(&u);
            if(download_stage==1&&(u.state==OFT_UDP_OFF||u.state==OFT_UDP_FAULT)){
                oft_ble_disconnect();download_stage=2;
            }
            oft_ble_snapshot_t b;oft_ble_snapshot(&b);oft_network_snapshot_t w;oft_network_snapshot(&w);
            /* A fixed150ms delay could race Wi-Fi BA/TX shutdown. Wait for the
               actual terminal state before forcing ROM download. */
            if(download_stage==2&&w.state==OFT_NETWORK_OFF&&
                (b.state==OFT_BLE_IDLE||b.state==OFT_BLE_OFF||b.state==OFT_BLE_FAULT)){
                REG_SET_BIT(LP_SYSTEM_REG_SYS_CTRL_REG, LP_SYSTEM_REG_FORCE_DOWNLOAD_BOOT);
                esp_restart();
            }
            if(esp_timer_get_time()>=download_deadline){
                printf("OFT_DOWNLOAD cancelled: radio shutdown did not finish\n");
                download_requested=false;download_stage=0;
            }
        }
        /* TinyUSB VFS is non-blocking. An explicit command lets the host retain
           every test result; USB enumeration alone is not a test trigger. */
        bool run_tests=false,run_dump=false;
        for(unsigned i=0;i<32;i++) {
            char c;if(read(STDIN_FILENO,&c,1)!=1)break;
            if(c=='\r')continue;
            if(c=='\n') {
                command[command_size]='\0';
                if(!discard_command && strcmp(command,"selftest")==0)run_tests=true;
                if(!discard_command && strcmp(command,"preview-dump")==0)run_dump=true;
                if(!discard_command && strcmp(command,"video-keyframe")==0)printf("OFT_VIDEO_REQUEST accepted=%d\n",oft_video_refresh(0));
                if(!discard_command && strcmp(command,"video-live")==0)printf("OFT_VIDEO_REQUEST accepted=%d interval_ms=5000\n",oft_video_refresh(5000));
                if(!discard_command && strcmp(command,"video-fast")==0)printf("OFT_VIDEO_REQUEST accepted=%d interval_ms=3000\n",oft_video_refresh(3000));
                if(!discard_command && strcmp(command,"video-quick")==0)printf("OFT_VIDEO_REQUEST accepted=%d interval_ms=2000\n",oft_video_refresh(2000));
                if(!discard_command && strcmp(command,"video-1500")==0)printf("OFT_VIDEO_REQUEST accepted=%d interval_ms=1500\n",oft_video_refresh(1500));
                if(!discard_command && strcmp(command,"video-1200")==0)printf("OFT_VIDEO_REQUEST accepted=%d interval_ms=1200\n",oft_video_refresh(1200));
                if(!discard_command && strcmp(command,"video-1000")==0)printf("OFT_VIDEO_REQUEST accepted=%d interval_ms=1000\n",oft_video_refresh(1000));
                if(!discard_command && strcmp(command,"video-off")==0)oft_video_stop_refresh();
                /* UI lifecycle diagnostics: switch pages only; never press Hold
                   or generate nonzero motion input. Uses the normal mode path. */
                if(!discard_command && strcmp(command,"video-show")==0)oft_udp_head_mode(true);
                if(!discard_command && strcmp(command,"ble-scan")==0)oft_ble_rescan();
                if(!discard_command && strcmp(command,"gauge-audit")==0)printf("OFT_GAUGE request=audit accepted=%d\n",oft_gauge_request(OFT_GAUGE_AUDIT));
                if(!discard_command && strcmp(command,"gauge-access")==0)printf("OFT_GAUGE request=access_restore accepted=%d\n",oft_gauge_request(OFT_GAUGE_ACCESS_RESTORE));
                if(!discard_command && strcmp(command,"gauge-apply")==0)printf("OFT_GAUGE request=apply accepted=%d\n",oft_gauge_request(OFT_GAUGE_APPLY));
                if(!discard_command && strcmp(command,"gauge-restore")==0)printf("OFT_GAUGE request=restore accepted=%d\n",oft_gauge_request(OFT_GAUGE_RESTORE));
                if(!discard_command && strcmp(command,"gauge-history")==0)printf("OFT_GAUGE request=history accepted=%d\n",oft_gauge_request(OFT_GAUGE_HISTORY));
                if(!discard_command && strcmp(command,"gauge-model")==0)printf("OFT_GAUGE request=model accepted=%d\n",oft_gauge_request(OFT_GAUGE_MODEL));
                if(!discard_command && strcmp(command,"settings-refresh")==0)oft_udp_settings_refresh();
                if(!discard_command && strcmp(command,"settings-video-test")==0)printf("OFT_SETTINGS_TEST accepted=%d\n",oft_udp_settings_test(0));
                if(!discard_command && strcmp(command,"settings-zoom-test")==0)printf("OFT_SETTINGS_TEST accepted=%d\n",oft_udp_settings_test(1));
                if(!discard_command && strcmp(command,"settings-photo-test")==0)printf("OFT_SETTINGS_TEST accepted=%d\n",oft_udp_settings_test(2));
                if(!discard_command && strcmp(command,"settings-photo-file-test")==0)printf("OFT_SETTINGS_TEST accepted=%d\n",oft_udp_settings_test(3));
                if(!discard_command && strcmp(command,"settings-photo-size-test")==0)printf("OFT_SETTINGS_TEST accepted=%d\n",oft_udp_settings_test(4));
                if(!discard_command && strcmp(command,"settings-hd-test")==0)printf("OFT_SETTINGS_TEST accepted=%d\n",oft_udp_settings_test(5));
                if(!discard_command && strcmp(command,"settings-27k-test")==0)printf("OFT_SETTINGS_TEST accepted=%d\n",oft_udp_settings_test(6));
                if(!discard_command && strcmp(command,"settings-mode-photo")==0)printf("OFT_SETTINGS_MODE accepted=%d\n",oft_udp_setting(OFT_SET_MODE,5,0));
                if(!discard_command && strcmp(command,"settings-mode-video")==0)printf("OFT_SETTINGS_MODE accepted=%d\n",oft_udp_setting(OFT_SET_MODE,1,0));
                if(!discard_command && strcmp(command,"video-fresh")==0)oft_video_latency_mode(true);
                if(!discard_command && strcmp(command,"video-balance")==0)oft_video_latency_mode(false);
                if(!discard_command && strcmp(command,"video-hide")==0)oft_udp_head_mode(false);
                if(!discard_command && strcmp(command,"video-trace-on")==0)printf("OFT_VIDEO_TRACE armed=%d\n",oft_video_trace_arm());
                if(!discard_command && strcmp(command,"video-trace")==0)oft_video_trace_dump();
                if(!discard_command && strcmp(command,"video-bench")==0)printf("OFT_BENCH_REQUEST accepted=%d\n",oft_video_benchmark());
                if(!discard_command && strcmp(command,"video-loss-test")==0)printf("OFT_VIDEO_LOSS_TEST accepted=%d\n",oft_video_loss_test());
                if(!discard_command && strcmp(command,"video-burst")==0)printf("OFT_VIDEO_BURST accepted=%d maximum=3 seconds=120\n",oft_video_burst_trial(3));
                if(!discard_command && strcmp(command,"video-burst6")==0)printf("OFT_VIDEO_BURST accepted=%d maximum=6 seconds=120\n",oft_video_burst_trial(6));
                if(!discard_command && strcmp(command,"video-idr")==0)oft_video_idr_only();
                if(!discard_command && strcmp(command,"ui-hold-test")==0)printf("OFT_UI_HOLD_REQUEST accepted=%d\n",oft_ui_hold_test());
                if(!discard_command && strcmp(command,"ui-menu-test")==0)printf("OFT_UI_MENU_REQUEST accepted=%d\n",oft_ui_menu_test());
                if(!discard_command && strcmp(command,"haptic-double-test")==0){oft_motion_head_ack(true);printf("OFT_HAPTIC_DOUBLE started=1 camera_input=0\n");}
                if(!discard_command && strcmp(command,"sample-dump")==0)oft_video_sample_dump();
                if(!discard_command){
                    /* Marker acknowledgement uses the MCU telemetry clock.
                       This command has no camera/control side effects. */
                    if(command_size>5&&strncmp(command,"mark-",5)==0){
                        bool safe=true;for(size_t j=5;j<command_size;j++)
                            if(!((command[j]>='a'&&command[j]<='z')||(command[j]>='A'&&command[j]<='Z')||(command[j]>='0'&&command[j]<='9')))safe=false;
                        if(safe)printf("OFT_MARK us=%lld name=%s\n",(long long)esp_timer_get_time(),command+5);
                    }
                    if(strcmp(command,"haptic-test")==0)oft_motion_haptic_test();
                    const char *probes[]={"probe-yaw+","probe-yaw-","probe-pitch+","probe-pitch-"};
                    for(unsigned j=0;j<4;j++)if(strcmp(command,probes[j])==0)
                        printf("OFT_PROBE request=%u accepted=%d\n",j,oft_udp_probe(j));
                }
                command_size=0;discard_command=false;
            } else if(command_size<sizeof(command)-1)command[command_size++]=c;
            else discard_command=true;
        }
        if(run_tests) {
            oft_udp_snapshot_t test_udp;oft_udp_snapshot(&test_udp);
            oft_video_snapshot_t test_video;oft_video_snapshot(&test_video);
            if(test_udp.state!=OFT_UDP_OFF||test_video.busy||heap_caps_get_free_size(MALLOC_CAP_SPIRAM)<3000000){
                printf("OFT_TEST_REFUSED disconnect camera and wait for decoder release first\n");
            }else{
                oft_protocol_run_selftests();
                protocol_tests_ran=true;
            }
        }
        if(run_dump)oft_video_dump();
        static oft_video_snapshot_t preview;oft_video_snapshot(&preview);
        static oft_udp_snapshot_t video_udp;oft_udp_snapshot(&video_udp);
        printf("OFT_MEMORY psram_mhz=%d cpu_mhz=%d\n",CONFIG_SPIRAM_SPEED,CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
        static oft_peripherals_snapshot_t peripheral;oft_peripherals_snapshot(&peripheral);
        printf("OFT_LOCAL_BATTERY valid=%d percent=%u mv=%u ma=%d design_mah=%u errors=%u presses=%u queued=%u\n",peripheral.battery_valid,peripheral.percent,peripheral.millivolts,peripheral.milliamps,peripheral.design_mah,peripheral.read_errors,peripheral.button_presses,peripheral.button_queued);
        printf("OFT_GAUGE_ESTIMATE configured=%d calibrated=0\n",peripheral.gauge_configured);
        static oft_settings_snapshot_t setting;oft_udp_settings_snapshot(&setting);
        printf("OFT_SETTINGS busy=%d sent=%u confirmed=%u errors=%u video=%d res=%02x fps=%u pairs=%u lens_valid=%d lens=%u photo=%d size=%u aspect=%u file=%u size_caps=%u file_caps=%u detail=%s\n",setting.busy,setting.sent,setting.confirmed,setting.errors,setting.values.video_valid,setting.values.resolution,setting.values.fps,setting.values.pair_count,setting.values.lens_valid,setting.values.lens,setting.values.photo_valid,setting.values.photo_size,setting.values.photo_aspect,setting.values.photo_file,setting.values.photo_size_count,setting.values.photo_file_count,setting.detail);
        printf("OFT_CAMERA mode=%u valid=%d recording=%d flags=%08lx age_ms=%lld shutter_sent=%u errors=%u busy=%d detail=%s\n",video_udp.camera.raw_mode,video_udp.camera.valid,video_udp.camera.recording,(unsigned long)video_udp.camera.flags,(long long)((esp_timer_get_time()-video_udp.camera.updated_us)/1000),video_udp.shutter_sent,video_udp.shutter_errors,video_udp.shutter_busy,video_udp.shutter_detail);
        printf("OFT_VIDEO_POLICY fresh=%d budget_ms=%u interval_ms=%u\n",preview.low_latency,preview.latency_budget_ms,preview.refresh_interval_ms);
        printf("OFT_VIDEO_BURST budget=%u p_decoded=%u last_idr=%lu\n",preview.burst_frames,preview.decoded_predicted_frames,(unsigned long)preview.last_idr_generation);
        printf("OFT_VIDEO_LATENCY output_age_ms=%u deadline_drops=%u\n",preview.output_age_ms,preview.deadline_drops);
        printf("OFT_CONVERT_TEST reference_us=%u mapped_us=%u\n",preview.convert_reference_us,preview.convert_mapped_us);
        printf("OFT_VIDEO_ADAPT selected_p=%u idr_ms=%u p_ms=%u\n",preview.selected_p_frames,preview.last_idr_decode_ms,preview.p_decode_ms);
        printf("OFT_VIDEO_OUTPUT direct=%u drained=%u timestamp_mismatch=%u\n",preview.direct_outputs,preview.drained_outputs,preview.timestamp_mismatches);
        printf("OFT_VIDEO_RECOVERY active=%d retry_after_ms=%u count=%u\n",preview.refresh_recovering,preview.retry_after_ms,preview.refresh_recoveries);
        printf("OFT_VIDEO_REFRESH requests=%u replies=%u failures=%u completed=%u auto=%d pending=%d\n",
            video_udp.keyframe_requests,video_udp.keyframe_replies,video_udp.keyframe_failures,
            preview.refresh_completed,preview.sparse_live,video_udp.keyframe_pending);
        printf("OFT_VIDEO_CODEC slice=%u ref=%u intra=%u idr=%u nonref=%u stack_free=%u internal_free=%u source_age_ms=%u\n",preview.slice_type,preview.nal_reference,preview.intra_units,preview.idr_units,preview.nonreference_units,preview.stack_free,(unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),preview.source_age_ms);
        printf("OFT_VIDEO decoded=%u presented=%u errors=%u au_drops=%u pending=%u bytes=%u age_ms=%u decode_ms=%u convert_ms=%u fps_milli=%u\n",
            preview.decoded_frames,preview.presented_frames,preview.decode_errors,preview.au_queue_drops,
            preview.queued_units,preview.queued_bytes,preview.queue_age_ms,preview.decode_ms,preview.convert_ms,preview.fps_milli);
        if(preview.codec_error[0])printf("OFT_PREVIEW_CODEC input_bytes=%u copy_errors=%u error=%s\n",preview.decode_input_bytes,preview.copy_errors,preview.codec_error);
        printf("OFT_PREVIEW ready=%d busy=%d packets=%u queue_drops=%u units=%u assembly_drops=%u invalid=%u attempts=%u mask=%08lx au_bytes=%u result=%d size=%ux%u decode_ms=%u psram_free=%u detail=%s\n",
            preview.ready,preview.busy,preview.packets,preview.queue_drops,preview.units,preview.assembly_drops,preview.invalid,preview.attempts,
            (unsigned long)preview.nal_mask,(unsigned)preview.au_bytes,preview.decoder_result,preview.width,preview.height,preview.decode_ms,
            (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),preview.detail);
        printf("OFT_BOOT stage=display board_error=%s target=esp32s31 cores=%u revision=%u uptime_ms=%" PRId64 " heap=%u min_heap=%u\n",
               esp_err_to_name(board_result),chip.cores, chip.revision, esp_timer_get_time()/1000,
               (unsigned)esp_get_free_heap_size(), (unsigned)esp_get_minimum_free_heap_size());
        oft_display_stats_t display=oft_board_display_stats();
        oft_touch_stats_t touch_stats;oft_touch_stats(&touch_stats);
        printf("OFT_INPUT_TIMING poll_max_us=%u errors=%u render_max_us=%u render_last_us=%u head_press=%u head_release=%u\n",(unsigned)touch_stats.max_gap_us,(unsigned)touch_stats.errors,(unsigned)display.render_max_us,(unsigned)display.render_last_us,(unsigned)touch_stats.head_presses,(unsigned)touch_stats.head_releases);
        printf("OFT_DISPLAY alignment_test=%d flushes=%" PRIu32 " unaligned=%" PRIu32 "\n",
               display.alignment_selftest_passed,display.flushes,display.unaligned_flushes);
        static oft_ble_snapshot_t ble;oft_ble_snapshot(&ble);
        printf("OFT_BLE init=%s state=%u choices=%u mtu=%u notifications=%u frames=%u writes=%u errors=%u disconnects=%u battery=%d\n",
            esp_err_to_name(radio_result),ble.state,ble.choice_count,ble.mtu,ble.notifications,ble.frames,
            ble.writes,ble.errors,ble.disconnects,ble.battery_valid?ble.battery:-1);
        printf("OFT_SCAN advertisements=%u dji=%u fff0=%u\n",ble.advertisements,ble.dji_advertisements,ble.fff0_advertisements);
        printf("OFT_BLE_REPLY matched=%u flags=%02x pairing_status=%u\n",ble.matched_replies,ble.last_reply_flags,ble.pairing_status);
        static oft_network_snapshot_t wifi;oft_network_snapshot(&wifi);
        printf("OFT_WIFI state=%u attempts=%u reason=%d\n",wifi.state,wifi.attempts,wifi.last_reason);
        static oft_udp_snapshot_t udp;oft_udp_snapshot(&udp);
        printf("OFT_UDP_DETAIL %s\n",udp.detail);
        printf("OFT_UDP state=%u port=%u packets=%u media=%u ack=%u max_ack_gap_us=%u registration=%u enable=%u heartbeat=%u/%u invalid=%u ambiguous=%u stack_free=%u\n",
            udp.state,udp.local_port,udp.packets,udp.media_packets,udp.acks,(unsigned)udp.max_ack_gap_us,udp.registrations,
            udp.enable_count,udp.heartbeat_replies,udp.heartbeat_sent,udp.invalid_packets,udp.ambiguous_status,(unsigned)udp.stack_free);
        printf("OFT_UDP_CONTROL nonzero=%u zero=%u failures=%u probe_done=%d raw_candidates=%d,%d,%d\n",
            udp.nonzero_commands,udp.zero_commands,udp.control_failures,udp.probe_done,udp.gimbal_raw[0],udp.gimbal_raw[1],udp.gimbal_raw[2]);
        if(udp.gimbal_updated_us){
            /* Bounded 1Hz raw snapshot outside the network callback. Preserve
               unknown fields, not just the currently selected candidates. */
            char hex[125];for(unsigned i=0;i<62;i++)snprintf(hex+i*2,3,"%02x",udp.gimbal_frame[i]);
            printf("OFT_GIMBAL_SNAPSHOT us=%lld duml=%s\n",(long long)udp.gimbal_updated_us,hex);
        }
        for(unsigned s=0;s<2;s++)if(udp.gimbal_status_size[s]){
            char hex[193];for(unsigned i=0;i<udp.gimbal_status_size[s];i++)snprintf(hex+i*2,3,"%02x",udp.gimbal_status_frame[s][i]);
            printf("OFT_GIMBAL_STATUS us=%lld duml=%s\n",(long long)udp.gimbal_status_us[s],hex);
        }
        printf("OFT_UDP_STOP receipt=%d first_zero_delay_us=%u\n",udp.zero_receipt_confirmed,(unsigned)udp.first_zero_delay_us);
        printf("OFT_INPUT live=%d state=%u mock_commands=%u mock_zeros=%u actions=%u\n",udp.live_enabled,udp.control_state,udp.mock_commands,udp.mock_zeros,udp.action_count);
        printf("OFT_HEAD mode=%d held=%d sensor_ready=%d relative_deg=%.3f,%.3f tracking=%d trial_clamped=%d camera_deg=%.3f,%.3f target_deg=%.3f,%.3f physical_gain_validated=0\n",udp.head_mode,udp.head_held,udp.head_sensor_ready,(double)udp.head_yaw_deg,(double)udp.head_pitch_deg,
            udp.head_tracking,udp.head_trial_guard,(double)udp.head_camera_yaw,(double)udp.head_camera_pitch,(double)udp.head_target_yaw,(double)udp.head_target_pitch);
        printf("OFT_HEAD_LIMIT mask=%u source=bench_soft_envelope\n",udp.head_limit_mask);
        printf("OFT_HEAD_DIRECTION yaw_held=%d\n",udp.head_yaw_held);
        printf("OFT_YAW_ALIGNMENT valid=%d shift_deg=%.3f\n",udp.head_yaw_aligned,(double)udp.head_yaw_shift);
        oft_rx_queue_stats_t rxq=oft_rx_queue_stats();
        printf("OFT_RXQ posts=%u drops=%u high_water=%u capacity=%u\n",rxq.posts,rxq.drops,rxq.high_water,rxq.capacity);
        static oft_motion_snapshot_t motion;oft_motion_snapshot(&motion);
        printf("OFT_IMU_INIT stage=%u result=%s\n",motion.imu_init_stage,esp_err_to_name(motion.imu_init_error));
        printf("OFT_IMU_RECOVERY ready=%d epoch=%u recoveries=%u bias_ready=%d event_drops=%u\n",motion.orientation_ready,motion.orientation_epoch,motion.recovery_count,motion.bias_ready,motion.event_drops);
        /* USB logging must not block the100Hz sensor/haptic task. These events
           keep their original MCU timestamps; overflow drops logs, not samples. */
        oft_motion_event_t event;
        for(unsigned i=0;i<32&&oft_motion_event_pop(&event);i++){
            if(event.gap)printf("orientation_gap us=%lld dt_us=%lld epoch=%u bias_retained=1\n",(long long)event.timestamp_us,(long long)event.interval_us,event.epoch);
            else printf("haptic_edge us=%lld on=%d\n",(long long)event.timestamp_us,event.on);
        }
        printf("OFT_IMU ok=%d id=%02x calibrated=%d samples=%u errors=%u a=%.4f,%.4f,%.4f g=%.4f,%.4f,%.4f q=%.5f,%.5f,%.5f,%.5f\n",
            motion.imu_ok,motion.imu_id,motion.bias_ready,motion.samples,motion.errors,
            (double)motion.accel[0],(double)motion.accel[1],(double)motion.accel[2],(double)motion.gyro[0],(double)motion.gyro[1],(double)motion.gyro[2],
            (double)motion.orientation.w,(double)motion.orientation.x,(double)motion.orientation.y,(double)motion.orientation.z);
        printf("OFT_MAG ok=%d,%d id=%02x,%02x uT=%.2f,%.2f,%.2f / %.2f,%.2f,%.2f motor=%d\n",
            motion.mag_ok[0],motion.mag_ok[1],motion.mag_id[0],motion.mag_id[1],
            (double)motion.mag_ut[0][0],(double)motion.mag_ut[0][1],(double)motion.mag_ut[0][2],
            (double)motion.mag_ut[1][0],(double)motion.mag_ut[1][1],(double)motion.mag_ut[1][2],motion.motor_on);
        printf("OFT_MAG_RAW valid=%u,%u raw=%d,%d,%d / %d,%d,%d rhall=%u,%u age_ms=%lld,%lld\n",
            motion.mag_valid_axes[0],motion.mag_valid_axes[1],
            motion.mag_raw[0][0],motion.mag_raw[0][1],motion.mag_raw[0][2],
            motion.mag_raw[1][0],motion.mag_raw[1][1],motion.mag_raw[1][2],
            motion.mag_rhall[0],motion.mag_rhall[1],
            (long long)((esp_timer_get_time()-motion.mag_sample_us[0])/1000),
            (long long)((esp_timer_get_time()-motion.mag_sample_us[1])/1000));
        if(protocol_tests_ran)printf("OFT_PROTOCOL tests=%u failures=%u ignored=%u\n",
            (unsigned)Unity.NumberOfTests,(unsigned)Unity.TestFailures,(unsigned)Unity.TestIgnores);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
