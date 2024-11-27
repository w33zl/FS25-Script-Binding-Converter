# ScriptBinding to Lua (by w33zl) - Converts a scriptBinding.xml file to a Lua file with LuaDoc annotation suitable fopr suggestions/type ahead in VS Code
VERSION = "1.1.0"

import re
import os
import sys
import xml.etree.ElementTree as ET

SUBSTITUE_FLOAT = True


def sanitize_variable_name(name):
    # Replace invalid characters with underscores
    # TODO Maybe just remove all invalid characters?
    name = re.sub(r"[^a-zA-Z0-9_]", "_", name)

    # Check if the name is a reserved keyword
    reserved_keywords = [
        "and", "break", "do", "else", "elseif", "end", "false", "for",
        "function", "goto", "if", "in", "local", "nil", "not", "or",
        "repeat", "return", "then", "true", "until", "while"
    ]
    if name in reserved_keywords:
        name += "Param"
    return name


def process_description(desc):
    # Add leading three dashes (---) after each newline to tag as LuaDoc comment
    # HACK maybe there is a more solid way to replace these (this will not work well with newline in input parameter descriptions)
    return desc.replace("&#xA;", "\n").replace("\n", "\n--- ")


def generate_lua_function(function_element):
    function_name = function_element.get("name")
    function_desc = process_description(function_element.get("desc"))
    lua_doc = f'--- {function_desc}\n'
    lua_params = []
    param_counter = 0
    for param in function_element.findall("./input/param"):
        param_name = param.get("name")
        param_counter += 1
        if not param_name:
            param_name = f"parameter{param_counter}"
            
        param_name = sanitize_variable_name(param_name)
        param_type = param.get("type")

        if SUBSTITUE_FLOAT and param_type == "float":
            param_type = "number"
        
        # check if value ends with ? (means optional)
        is_optional = param_type.endswith("?")
        if is_optional:
            # add "|nil" to the end of the type to make it optional
            param_type = param_type[:-1] + "|nil"

        param_desc = process_description(param.get("desc"))
        param_doc = f'---@param {param_name} {param_type}'
        if param_desc:
            param_doc += f' "{param_desc}"'
        lua_params.append(param_doc)

    # Extract return parameter info from the <output> node
    output_param = function_element.find("./output/param")
    if output_param is not None:
        return_name = sanitize_variable_name(output_param.get("name"))
        return_type = output_param.get("type")
        return_desc = process_description(output_param.get("desc"))
        if return_name:
            return_name = f"{return_name} "

        lua_return = f'---@return {return_name}{return_type}'
        if return_desc:
            lua_return += f' "{return_desc}"'
        lua_params.append(lua_return)

    lua_params_str = "\n".join(lua_params)
    lua_function = f'\nfunction {function_name}({", ".join(sanitize_variable_name(param.get("name")) if param.get("name") else f"parameter{idx + 1}" for idx, param in enumerate(function_element.findall("./input/param")))}) end\n\n'
    return lua_doc + lua_params_str + lua_function


def convert_xml_to_lua(xml_file_path, lua_file_path):
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    lua_code = ""
    for function_element in root.findall("./function"):
        lua_code += generate_lua_function(function_element)

    with open(lua_file_path, "w") as f:
        # print header with script name and VERSION to the file
        f.write(f"--[[\n\nScriptBindingToLua v{VERSION} - Converts a scriptBinding.xml file to a Lua file with LuaDoc annotation suitable fopr suggestions/type ahead in VS Code\n\n")
        f.write(f"Author: w33zl / WZL Modding\n")
        f.write(f"GitHub: github.com/w33zl/FS25-Script-Binding-Converter\n\n")

        # check if a file name VERSION exists in the user's \Documents\My Games\FarmingSimulator2025\ folder, and if exists, print text with "Game version: " and the content of the VERSION file
        try:
            with open(f"{os.environ['USERPROFILE']}\\Documents\\My Games\\FarmingSimulator2025\\VERSION", "r") as version_file:
                version = version_file.read().strip()
                f.write(f"Game version: {version}\n\n")
        except FileNotFoundError:
            f.write("Game version: Not found\n\n")

        f.write("]]\n\n")

        # Add aliases before code
        f.write("---@alias entityId number\n")
        f.write("---@alias float number\n")
        f.write("\n")

        # write the lua code to the file
        f.write(lua_code)

if __name__ == "__main__":
    # check if first argument is a file path, if not, use default path
    input_file = "scriptBinding.xml"
    output_file = "scriptBinding.lua"

    if len(sys.argv) > 1:
        # check if second argument is a file path, use that for the lua output file, othwerwise use default
        input_file = sys.argv[1]
        if len(sys.argv) > 2:
            output_file = sys.argv[2]
            # convert_xml_to_lua(sys.argv[1], sys.argv[2])
        else:
            output_file = sys.argv[1].replace(".xml", ".lua")
            # convert_xml_to_lua(sys.argv[1], sys.argv[1].replace(".xml", ".lua"))
    # else:
    #     convert_xml_to_lua("scriptBinding.xml", "scriptBinding.lua")

    convert_xml_to_lua(input_file, output_file)
