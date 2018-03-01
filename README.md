# SSD1322 display driver

- `ssd1322.c` is a double framebuffer spi driver (256x64)
- `ssd1322.py` is a python display class

ssd1322.service:
```
[Unit]
Description=SSD1322 Display Driver

[Service]
ExecStart=/usr/local/sbin/ssd1322

[Install]
WantedBy=multi-user.target
```

Python example:
```python
import cairo
import ssd1322

with ssd1322.ssd1322() as display:
    context = cairo.Context(display.surface)

    context.set_source_rgb(0, 0, 0)
    context.paint()

    context.move_to(30, 45)
    context.set_font_size(30)
    context.set_source_rgb(1, 1, 1)
    context.show_text('Hello world!')

    display.update()
```
