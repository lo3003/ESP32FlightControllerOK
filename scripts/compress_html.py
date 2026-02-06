import gzip
import os
import sys

def main():
    input_path = os.path.join("web", "index.html")
    output_path = os.path.join("include", "html_page_gz.h")

    with open(input_path, "r", encoding="utf-8") as f:
        html_content = f.read()

    compressed = gzip.compress(html_content.encode("utf-8"), compresslevel=9)

    with open(output_path, "w") as f:
        f.write("// Auto-generated — do not edit manually\n")
        f.write("// Source: web/index.html\n")
        f.write(f"// Original size: {len(html_content)} bytes\n")
        f.write(f"// Compressed size: {len(compressed)} bytes\n")
        f.write(f"// Compression ratio: {len(compressed)*100//len(html_content)}%\n\n")
        f.write("#pragma once\n")
        f.write("#include <pgmspace.h>\n\n")
        f.write(f"const size_t html_page_gz_len = {len(compressed)};\n\n")
        f.write("const uint8_t html_page_gz[] PROGMEM = {\n")

        for i in range(0, len(compressed), 16):
            chunk = compressed[i:i+16]
            hex_str = ", ".join(f"0x{b:02x}" for b in chunk)
            f.write(f"    {hex_str},\n")

        f.write("};\n")

    print(f"Compressed {len(html_content)} -> {len(compressed)} bytes ({len(compressed)*100//len(html_content)}%)")

# Support both standalone execution and PlatformIO pre-build script
try:
    Import("env")
    # Running as PlatformIO script — run from project dir
    import os
    project_dir = env.subst("$PROJECT_DIR")
    os.chdir(project_dir)
    main()
except NameError:
    # Running standalone (python scripts/compress_html.py)
    if __name__ == "__main__":
        main()
