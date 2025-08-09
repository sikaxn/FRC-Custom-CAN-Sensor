For PCB. 


# ESPTool merge fw for web tool

```
esptool --chip esp32 merge_bin -o seal_rgb_webtool.bin `
  --flash_mode dio --flash_freq 80m --flash_size 4MB `
  0x1000   seal_rgb.ino.bootloader.bin `
  0x8000   seal_rgb.ino.partitions.bin `
  0x10000  seal_rgb.ino.bin
```

# JSON for web tool

```
{
  "name": "Factory Test",
  "version": "1.0.0",
  "builds": [
    {
      "chipFamily": "ESP32",
      "improv": false,
      "parts": [
        {
          "path": "../firmware/factory.bin",
          "offset": 0
        }
      ]
    }
  ]
}
```