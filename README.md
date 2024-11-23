# FS25 Script Binding Converter

Converts a scriptBinding.xml file to a Lua file with LuaDoc annotation suitable fopr suggestions/type ahead in VS Code

## Usage
`python ScriptBindingToLua.py [input_file] [output_file]`

### Arguments
- **input_file** _(optional)_: The path to the scriptBinding.xml file to convert. Defaults to "scriptBinding.xml".
- **output_file** _(optional)_: The path to the output Lua file. Defaults to "scriptBinding.lua".

## Example
`python ScriptBindingToLua.py C:\FS25\debugger\scriptBinding.xml C:\MyProjects\scriptBinding.lua`

## Note
The output file will be overwritten if it already exists.

