# libssc backend (`libsensors_libssc.so`)

Out-of-tree backend of the [mainline Sensors HAL](../../../common/interfaces/sensors/mainline/README.md)
for sensors managed by the Qualcomm Sensor Core (SLPI/ADSP) on SoCs from 2018
onwards, where the application processor has no direct access to the sensor
hardware. It uses [libssc](https://libssc.dylanvanassche.be) (GLib based) to
talk to the DSP over QMI/QRTR.

## Sensors

| libssc sensor  | Android type      | Notes                                                        |
|----------------|-------------------|--------------------------------------------------------------|
| accelerometer  | `ACCELEROMETER`   | m/s², mount matrix from the DSP applied by libssc            |
| gyroscope      | `GYROSCOPE`       | rad/s                                                        |
| magnetometer   | `MAGNETIC_FIELD`  | µT                                                           |
| light          | `LIGHT`           | lux, on-change                                               |
| proximity      | `PROXIMITY`       | near → 0 cm, far → 5 cm, on-change, wake-up                  |
| compass        | `ORIENTATION`     | azimuth in degrees from the DSP rotation vector; pitch/roll 0 |

Sensor name, vendor and sample rate come from the attributes reported by the
DSP. The DSP streams continuous sensors at a single fixed rate; slower rates
requested by the framework are obtained by decimation.

## Threading

libssc emits its `measurement` signals from the GLib default main context and
its `*_sync()` helpers iterate that context while waiting. `GlibWorker` runs the
default context on a dedicated thread; every libssc call (`Initialize`,
`Activate`, `Deinitialize`) is executed on that thread through
`GlibWorker::Invoke()`, and measurements are forwarded to the frontend from it.

## Configuration

Keys are read through the HAL settings mechanism (property
`vendor.sensors.<key>` or `/{odm,vendor}/etc/sensors/*.conf`). `<kind>` is one
of `accel`, `gyro`, `magn`, `light`, `proximity`, `compass`.

| Key                          | Meaning                                                            |
|------------------------------|--------------------------------------------------------------------|
| `ssc.sensors`                | Comma separated list of kinds to expose (default: all)             |
| `ssc.discovery_wait_ms`      | Keep retrying discovery for this long while the DSP boots (default 0) |
| `ssc.<kind>.name`            | Sensor name                                                        |
| `ssc.<kind>.vendor`          | Vendor string                                                      |
| `ssc.<kind>.power`           | Power estimate (mA)                                                |
| `ssc.<kind>.max_range`       | Range in Android units                                             |
| `ssc.<kind>.resolution`      | Resolution in Android units                                        |
| `ssc.<kind>.min_delay_us`    | Fastest sampling period                                            |
| `ssc.<kind>.max_delay_us`    | Slowest sampling period                                            |
| `ssc.<kind>.wake_up`         | Override the wake-up flag                                          |
| `ssc.<kind>.mount_matrix`    | Extra matrix applied on top of the DSP mount matrix (3-axis kinds) |

## Build and installation

```makefile
$(call soong_config_set_bool,libsensors_libssc,enabled,true)
$(call soong_config_set_string_list,sensors_hal_mainline,include_custom_backends,//hardware/mainline/qcom:libsensors_libssc)
$(call soong_config_set,sensors_hal_mainline,load_custom_backends,libssc$(comma)iio$(comma)input$(comma)mock)
```

The library can also be installed on `/vendor` instead of being bundled into
the APEX; the HAL searches `/vendor/lib{,64}{/hw,}`. The backend list can be
changed at runtime with `setprop vendor.sensors.backends libssc,iio`.

## Requirements

* The sensor DSP must be up: remoteproc firmware, `pd-mapper`, `qrtr` and the
  `fastrpc`/QMI plumbing of the platform.
* SELinux: `hal_sensors_default` needs `AF_QIPCRTR` socket access
  (`qipcrtr_socket`) for QMI over QRTR.
