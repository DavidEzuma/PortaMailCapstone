# Minimal C++ External -> LCD Communication

This is a minimal C++ example for talking to the PortaMail LCD server.

## Build
```bash
cd LCD_web/portamail_ui/examples/external_cpp_minimal
c++ -std=c++17 external_lcd_minimal.cpp -lcurl -o external_lcd_minimal
```

## Use
Default LCD URL is `http://127.0.0.1:5050`.

```bash
# read current LCD state
./external_lcd_minimal state

# tell LCD robot arrived
./external_lcd_minimal arrived

# tell LCD robot is dock idle
./external_lcd_minimal dock

# poll edge events incrementally (since_ts)
./external_lcd_minimal poll
```

## Does this work with C++?
Yes. The LCD API is plain HTTP + JSON, so C++, Python, C, Rust, etc. all work.
This example uses `libcurl` only and does not require a JSON library.
