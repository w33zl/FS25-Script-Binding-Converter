# ScriptBinding to Lua (by w33zl) - Converts a scriptBinding.xml file to a Lua file with LuaDoc annotation suitable fopr suggestions/type ahead in VS Code

import re
import xml.etree.ElementTree as ET

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
        f.write(lua_code)

if __name__ == "__main__":
    convert_xml_to_lua("scriptBinding.xml", "scriptBinding.lua")
