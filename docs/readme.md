# Documentation

## TODOs

- Understand why creating file `sysbuild/mcuboot.overlay` breaks the build:

```dts
&can1 {
    status = "disabled";
};

&i2c1 {
    status = "disabled";
};

&usb {
    status = "disabled";
};
```