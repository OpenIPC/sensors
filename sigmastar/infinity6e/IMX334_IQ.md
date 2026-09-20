# Factory IMX334 daytime IQ

imx334.bin was extracted from the original firmware of the SSC338Q / IMX334
camera used for the driver port. It is not an independently authored profile.
Public redistribution authorization has not been established. Maintainer review
is requested; this submission does not claim the repository license relicenses
the binary. No firmware dump, credentials or driver changes are included here.

Size: 86,944 bytes. SHA-256:
`cb267960d60c9c646516255c5d6c80f8991ce8efd861ae7c69974a8c29f24e92`.
The file matches the project's RC5 release and the camera's checked IQ file.

For a compatible firmware, install as /etc/sensors/imx334.bin and explicitly set:

```yaml
isp:
  sensorConfig: /etc/sensors/imx334.bin
```

This explicit setting avoided Majestic automatically choosing imx335.bin after
video-service restarts. All six tested linear mode transitions loaded this IQ
successfully. That validates loading, not exhaustive image-quality tuning.
HDR suitability and HDR image output remain unverified. The IMX334 driver is
proposed separately in OpenIPC/sensors#5; this PR does not depend on its commits.
