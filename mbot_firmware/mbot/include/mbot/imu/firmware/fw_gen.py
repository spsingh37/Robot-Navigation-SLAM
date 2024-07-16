import argparse

def create_header_file(firmware_file_path, output_header_file_path, array_name):
    # Read binary data
    with open(firmware_file_path, 'rb') as f:
        byte_array = f.read()

    # Start creating output string
    output_string = f"const unsigned char {array_name}[] = {{\n"

    # Convert each byte to hexadecimal and add it to the output string
    for i, byte in enumerate(byte_array):
        output_string += "0x{:02x}, ".format(byte)
        if (i + 1) % 12 == 0:  # Every 12 bytes, add a line break
            output_string += "\n"

    output_string += "};\n"  # Close the array

    # Write the output string to the header file
    with open(output_header_file_path, 'w') as f:
        f.write(output_string)

if __name__ == "__main__":
    # Create argument parser
    parser = argparse.ArgumentParser(description='Convert firmware file to C header file')
    parser.add_argument('firmware_file_path', type=str, help='Path to firmware file')
    parser.add_argument('output_header_file_path', type=str, help='Path to output header file')
    parser.add_argument('--array_name', type=str, default='bhy1_fw', help='Name of the byte array in the C header file')

    # Parse arguments
    args = parser.parse_args()

    # Use function to convert a firmware file to a C header
    create_header_file(args.firmware_file_path, args.output_header_file_path, args.array_name)
