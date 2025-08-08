import argparse
import os

def convert_bin_to_c_array(input_file, output_file, array_name="synsense_config_array"):
    """Converts a binary file to a C-style byte array header file."""
    try:
        with open(input_file, "rb") as f:
            binary_data = f.read()
    except FileNotFoundError:
        print(f"Error: Input file not found at {input_file}")
        return

    c_code = f"#ifndef {array_name.upper()}_H\n"
    c_code += f"#define {array_name.upper()}_H\n\n"
    c_code += f"const unsigned char {array_name}[] = {{\n  "

    for i, byte in enumerate(binary_data):
        c_code += f"0x{byte:02x}, "
        if (i + 1) % 16 == 0:
            c_code += "\n  "

    c_code = c_code.strip().rstrip(',')
    c_code += "\n};\n\n"
    c_code += f"const unsigned int {array_name}_len = sizeof({array_name});\n\n"
    c_code += f"#endif // {array_name.upper()}_H\n"

    try:
        with open(output_file, "w") as f:
            f.write(c_code)
        print(f"Successfully converted {input_file} to {output_file}")
    except IOError:
        print(f"Error: Could not write to output file {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a binary config file to a C header file.")
    parser.add_argument("input", help="Path to the input binary file.")
    parser.add_argument("-o", "--output", help="Path to the output C header file.", default="config_array.h")
    parser.add_argument("-n", "--name", help="Name for the C array.", default="config_data")
    args = parser.parse_args()

    convert_bin_to_c_array(args.input, args.output, args.name)
