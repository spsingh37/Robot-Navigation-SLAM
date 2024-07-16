import os
import sys
import argparse
import re
from collections import defaultdict

# Parse an LCM file and return package name, struct name, and fields.
def parse_lcm_file(file_path):
    """
    \brief Parse an LCM file to extract package name, struct name, and fields.
    
    \param[in] file_path The path to the LCM file.
    
    \return A tuple containing package_name, struct_name, and fields.
    """
    with open(file_path, 'r') as f:
        content = f.read()

    package_name = re.search(r'package\s+(\w+);', content).group(1)
    struct_name = re.search(r'struct\s+(\w+)', content).group(1)

    # Remove lines containing 'package' or 'struct'
    content = re.sub(r'^(?:package|struct).*$', '', content, flags=re.MULTILINE)

    # Updated regex pattern to match fields
    fields = re.findall(r'\s*(\w+)\s+(\w+)(\[(?:\d+|\w+)\])?;\s*(//.*)?', content)
    has_vla = False
    for field_type, field_name, array_size, comment in fields:
        if array_size != '' and not array_size.strip('[]').isdigit():
            has_vla = True
    
    return package_name, struct_name, fields, has_vla


# Generate C struct and serialize/deserialize functions from LCM fields.
def generate_c_struct(package_name, struct_name, fields, max_string_length=256):
    """
    \brief Generate C struct and serialize/deserialize functions from LCM fields.
    
    \param[in] package_name The package name.
    \param[in] struct_name The struct name.
    \param[in] fields The list of fields in the LCM struct.
    \param[in] max_string_length The maximum string length for string fields (default: 256).

    \return A string containing the C struct and serialize/deserialize functions.
    """
    c_struct = f'typedef struct __attribute__((__packed__)) serial_{struct_name} {{\n'

    lcm_standard_types = ['int8_t', 'int16_t', 'int32_t', 'int64_t', 'float', 'double', 'string', 'boolean', 'byte']

    for field_type, field_name, array_size, comment in fields:

        # Replace LCM string fields with fixed-size character arrays in C
        if field_type == 'string':
            field_type = 'char'
            array_size = f'[{max_string_length}]'
        # Replace LCM boolean fields with bool in C
        elif field_type == 'boolean':
            field_type = 'bool'
        # Replace LCM byte fields with uint8_t in C
        elif field_type == 'byte':
            field_type = 'uint8_t'
        # Prepend 'serial_' to custom LCM types in C
        elif field_type not in lcm_standard_types:
            field_type = f'serial_{field_type}'

        c_struct += f'    {field_type} {field_name}{f"{array_size}" if array_size else ""};'
        if comment:
            c_struct += f' {comment}'
        c_struct += '\n'

    c_struct += f'}} serial_{struct_name};\n\n'

    # Add function prototypes for serialize and deserialize functions
    c_struct += generate_function_protoypes(struct_name)

    return c_struct

def generate_function_protoypes(struct_name):
    """
    \brief Generate C function prototypes for serializing and deserializing
    a given LCM struct type.

    \param[in] struct_name The name of the LCM struct type to generate functions for.

    \return A string containing the C function prototypes for serializing
    and deserializing the LCM struct type.
    """
    # Generate deserialize function prototype
    deserialize_proto = f'static inline int {struct_name}_deserialize(uint8_t* src, serial_{struct_name}* dest);'

    # Generate serialize function prototype
    serialize_proto = f'static inline int {struct_name}_serialize(serial_{struct_name}* src, uint8_t* dest);'

    return deserialize_proto + '\n' + serialize_proto + '\n'


def generate_serialize_deserialize_functions(struct_name):
    """
    \brief Generate C function code for serializing and deserializing
    a given LCM struct type.

    \param[in] struct_name The name of the LCM struct type to generate functions for.

    \return A string containing the C function code for serializing
    and deserializing the LCM struct type.
    """
    # Generate deserialize function
    deserialize = f'int {struct_name}_deserialize(uint8_t* src, serial_{struct_name}* dest) {{\n'
    deserialize += f'    memcpy(dest, src, sizeof(serial_{struct_name}));\n'
    deserialize += f'    return 1;\n'
    deserialize += f'}}\n\n'

    # Generate serialize function
    serialize = f'int {struct_name}_serialize(serial_{struct_name}* src, uint8_t* dest) {{\n'
    serialize += f'    memcpy(dest, src, sizeof(serial_{struct_name}));\n'
    serialize += f'    return 1;\n'
    serialize += f'}}\n'

    return deserialize + serialize


def process_lcm_files(folder_path, ordered_lcm_files):
    """
    \brief Process LCM files in a folder, generate C structs and functions, and save them in header files.
    
    \param[in] folder_path The path to the folder containing LCM files.
    \param[in] ordered_lcm_files List of LCM files in the order they should be processed.

    The function will create one header file and one C file for each unique package name found in the LCM files. The header
    files will be named {package_name}_serial.h and will contain the C structs and serialize/deserialize
    function prototypes for each LCM struct in the package. The C files will be named {package_name}_serial.c and will
    contain the serialize/deserialize function implementations.
    """
    lcm_files = ordered_lcm_files   

    package_structs = defaultdict(list)
    package_types = defaultdict(list)
    package_funcs = defaultdict(list)

    for lcm_file in lcm_files:
        c_struct = None
        c_funcs = None
        file_path = os.path.join(folder_path, lcm_file)
        package_name, struct_name, fields, has_vla = parse_lcm_file(file_path)
        if(has_vla):
            print(f"skipping {struct_name} from {package_name}, contains variable length array")
            continue
        
        c_struct = generate_c_struct(package_name, struct_name, fields)
        c_funcs = generate_serialize_deserialize_functions(struct_name)
        
        # Group structs and functions by package name
        if c_struct is not None and c_funcs is not None:
            package_structs[package_name].append(c_struct)
            package_funcs[package_name].append(c_funcs)
            package_types[package_name].append(struct_name)

    for package_name in package_structs.keys():
        # Generate header file for each package
        header_file_name = f'{package_name}_serial.h'
        with open(header_file_name, 'w') as f:
            # Generate multiline comment at top of file with list of included types
            type_list = '\n *   '.join(package_types[package_name])
            f.write(f"""/*
 * This header file was autogenerated by the LCM to C header file generator.
 * Included LCM types:
 *   {type_list}
 */\n\n""")
            f.write("#ifndef SERIAL_MSG_H\n")
            f.write("#define SERIAL_MSG_H\n")
            f.write("#pragma once\n")
            f.write("#include <stdint.h>\n")
            f.write("#include <string.h>\n\n")
            f.write("#include <stdbool.h>\n\n")
            for struct in package_structs[package_name]:
                f.write(struct)
                f.write("\n\n")
            for func in package_funcs[package_name]:
                f.write(func)
                f.write("\n\n")
            f.write("#endif\n")
            f.close()
        
        # # Generate .c file for each package
        # c_file_name = f'{package_name}_serial.c'
        # with open(c_file_name, 'w') as f:
        #     f.write("#include \"" + header_file_name + "\"\n\n")
        #     for funcs in package_funcs[package_name]:
        #         f.write(funcs)
        #         f.write("\n\n")
        #     f.close()

def main():
    folder_path = sys.argv[1]
    ordered_lcm_files = sys.argv[2:]

    if not os.path.isdir(folder_path):
        print(f"Error: {folder_path} is not a valid directory")
        exit(1)

    process_lcm_files(folder_path, ordered_lcm_files)  

if __name__ == "__main__":
    main()