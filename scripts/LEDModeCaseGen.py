def generate_cases_to_file(filename="generated_cases.txt", start=4, end=254):
    code_block = """\
    case {i}: {{
      // Mode {i}: solid color (customizable)
      if (canOnOff) {{
        CRGB color = CRGB{{canR, canG, canB}};
        color.nscale8_video(canBrig);
        fill_solid(leds, NUM_LEDS, color);
      }} else {{
        fill_solid(leds, NUM_LEDS, CRGB::Black);
      }}
      FastLED.show();
      break;
    }}
"""

    with open(filename, "w") as f:
        for i in range(start, end + 1):
            f.write(code_block.format(i=i))

    print(f"Cases written to {filename}")

if __name__ == "__main__":
    generate_cases_to_file()
