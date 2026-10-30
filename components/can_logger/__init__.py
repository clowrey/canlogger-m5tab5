"""ESPHome external component: Tab5 CAN logger (real-time capture engine).

Usage in YAML:

    external_components:
      - source: { type: local, path: <path to>/tab5-canlog/components }
        components: [can_logger]

    can_logger:
      id: canlog
      rx_pin: GPIO54        # from CA-IS3050G CRX (Tab5 Port A white wire)
      # tx_pin omitted -> listen-only with no TX pin: cannot drive the bus
      bitrate: 500000
      sd_path: /sdcard      # mounted by this component (SDMMC slot 0)
      ring_mb: 16
      seg_bytes: 67108864   # start a new segNNNN.canbin every 64 MB
      fd_channel:           # optional second, CAN FD channel; omit for classic only
        driver: gs_usb      # CANable 2.0 with Elmue's firmware on the USB-A port
        bitrate: 500000
        data_bitrate: 2000000   # 0 = classic CAN on this channel

The SD pins default to the Tab5's (slot 0: CLK 43, CMD 44, D0-D3 39-42, IO
power from on-chip LDO 4, as in M5's Tab5 BSP).
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import esp32, web_server_base
from esphome.components.web_server_base import CONF_WEB_SERVER_BASE_ID
from esphome.const import CONF_ID

DEPENDENCIES = ["esp32", "psram"]

CONF_TX_PIN = "tx_pin"
CONF_RX_PIN = "rx_pin"
CONF_BITRATE = "bitrate"
CONF_SD_PATH = "sd_path"
CONF_RING_MB = "ring_mb"
CONF_SEG_BYTES = "seg_bytes"
CONF_HTTP_PORT = "http_port"
CONF_SD_CLK_PIN = "sd_clk_pin"
CONF_SD_CMD_PIN = "sd_cmd_pin"
CONF_SD_D0_PIN = "sd_d0_pin"
CONF_SD_D1_PIN = "sd_d1_pin"
CONF_SD_D2_PIN = "sd_d2_pin"
CONF_SD_D3_PIN = "sd_d3_pin"
CONF_SD_LDO_CHANNEL = "sd_ldo_channel"
CONF_FD_CHANNEL = "fd_channel"
CONF_DRIVER = "driver"
CONF_DATA_BITRATE = "data_bitrate"
CONF_SAMPLE_POINT = "sample_point"
CONF_DATA_SAMPLE_POINT = "data_sample_point"
CONF_TEST_LOOPBACK_FRAMES = "test_loopback_frames"

FD_CHANNEL_SCHEMA = cv.Schema(
    {
        # gs_usb: a CANable 2.0 (STM32G4) with Elmue's CANable 2.5 firmware on
        # the USB-A host port. (An MCP2518FD on SPI is planned, docs/CANFD-PLAN.md.)
        cv.Optional(CONF_DRIVER, default="gs_usb"): cv.one_of("gs_usb", lower=True),
        cv.Optional(CONF_BITRATE, default=500000): cv.int_range(min=10000, max=1000000),
        cv.Optional(CONF_DATA_BITRATE, default=2000000): cv.Any(
            cv.one_of(0, int=True), cv.int_range(min=500000, max=8000000)
        ),
        cv.Optional(CONF_SAMPLE_POINT, default="80%"): cv.percentage,
        cv.Optional(CONF_DATA_SAMPLE_POINT, default="75%"): cv.percentage,
        # Desk test only: the adapter goes into internal loopback (it never drives
        # the bus) and the Tab5 sends this many numbered frames through it.
        cv.Optional(CONF_TEST_LOOPBACK_FRAMES, default=0): cv.int_range(min=0, max=100000000),
    }
)

can_logger_ns = cg.esphome_ns.namespace("can_logger")
CanLogger = can_logger_ns.class_("CanLogger", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(CanLogger),
        cv.Required(CONF_RX_PIN): pins.internal_gpio_input_pin_number,
        cv.Optional(CONF_TX_PIN): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_BITRATE, default=500000): cv.one_of(
            125000, 250000, 500000, 1000000, int=True
        ),
        cv.Optional(CONF_SD_PATH, default="/sdcard"): cv.string,
        cv.Optional(CONF_RING_MB, default=16): cv.int_range(min=1, max=28),
        cv.Optional(CONF_SEG_BYTES, default=67108864): cv.positive_int,
        cv.Optional(CONF_HTTP_PORT, default=80): cv.port,
        cv.Optional(CONF_SD_CLK_PIN, default="GPIO43"): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_SD_CMD_PIN, default="GPIO44"): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_SD_D0_PIN, default="GPIO39"): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_SD_D1_PIN, default="GPIO40"): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_SD_D2_PIN, default="GPIO41"): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_SD_D3_PIN, default="GPIO42"): pins.internal_gpio_output_pin_number,
        cv.Optional(CONF_SD_LDO_CHANNEL, default=4): cv.int_range(min=0, max=4),
        cv.Optional(CONF_FD_CHANNEL): FD_CHANNEL_SCHEMA,
        # With ESPHome's web_server configured, /files and /dl are served there
        # (its port and auth) and http_port is unused; HA then links the device.
        cv.OnlyWith(CONF_WEB_SERVER_BASE_ID, "web_server_base"): cv.use_id(
            web_server_base.WebServerBase
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


def _require_filesystem(config):
    # Must run at validation time: esp32's to_code writes the FATFS/VFS options
    # before ours runs. Without these ESPHome forces CONFIG_FATFS_LFN_NONE=y (our
    # file names are not 8.3) and disables VFS dir support (opendir/readdir for
    # the HTTP file listing always fail).
    esp32.require_fatfs()
    esp32.require_vfs_dir()
    return config


FINAL_VALIDATE_SCHEMA = _require_filesystem


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_rx_pin(config[CONF_RX_PIN]))
    if CONF_TX_PIN in config:
        cg.add(var.set_tx_pin(config[CONF_TX_PIN]))
    cg.add(var.set_bitrate(config[CONF_BITRATE]))
    cg.add(var.set_sd_path(config[CONF_SD_PATH]))
    cg.add(var.set_ring_mb(config[CONF_RING_MB]))
    cg.add(var.set_seg_bytes(config[CONF_SEG_BYTES]))
    cg.add(var.set_http_port(config[CONF_HTTP_PORT]))
    cg.add(
        var.set_sd_pins(
            config[CONF_SD_CLK_PIN],
            config[CONF_SD_CMD_PIN],
            config[CONF_SD_D0_PIN],
            config[CONF_SD_D1_PIN],
            config[CONF_SD_D2_PIN],
            config[CONF_SD_D3_PIN],
        )
    )
    cg.add(var.set_sd_ldo_channel(config[CONF_SD_LDO_CHANNEL]))
    if CONF_WEB_SERVER_BASE_ID in config:
        cg.add_define("USE_CAN_LOGGER_WEB")
        base = await cg.get_variable(config[CONF_WEB_SERVER_BASE_ID])
        cg.add(var.set_web_server_base(base))

    # ESPHome strips these IDF components by default (TWAI is only pulled in by
    # esp32_can, FAT is "unused"); we need the node TWAI driver and FAT on SD.
    esp32.include_builtin_idf_component("esp_driver_twai")
    esp32.include_builtin_idf_component("fatfs")
    esp32.include_builtin_idf_component("wear_levelling")
    # segNNNN.canbin is not an 8.3 name
    esp32.add_idf_sdkconfig_option("CONFIG_FATFS_LFN_HEAP", True)
    # RX ISR keeps running while the flash cache is off (SPI-flash writes, OTA)
    esp32.add_idf_sdkconfig_option("CONFIG_TWAI_ISR_IN_IRAM", True)
    esp32.add_idf_sdkconfig_option("CONFIG_TWAI_ISR_CACHE_SAFE", True)
    # The P4 heap can place the 3.5 KB main-task stack in its 8 KB SPM (TCM);
    # the first flash mapping (ESPHome opening NVS) then asserts in
    # esp_task_stack_is_sane_cache_disabled() and the board boot-loops. Seen
    # with the fd_channel build. A stack bigger than the SPM can't land there.
    esp32.add_idf_sdkconfig_option("CONFIG_ESP_MAIN_TASK_STACK_SIZE", 9216)

    if fd := config.get(CONF_FD_CHANNEL):
        cg.add_define("USE_CAN_LOGGER_FD")
        cg.add_define("USE_CAN_LOGGER_GSUSB")
        cg.add(
            var.set_fd_gs_usb(
                fd[CONF_BITRATE],
                fd[CONF_DATA_BITRATE],
                fd[CONF_SAMPLE_POINT],
                fd[CONF_DATA_SAMPLE_POINT],
                fd[CONF_TEST_LOOPBACK_FRAMES],
            )
        )
        # USB host library (built into IDF 5.x, stripped unless asked for);
        # more of the controller's FIFO for IN transfers
        esp32.include_builtin_idf_component("usb")
        esp32.add_idf_sdkconfig_option("CONFIG_USB_HOST_HW_BUFFER_BIAS_IN", True)
