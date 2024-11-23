# FS25 Script Binding Converter

Converts a scriptBinding.xml file to a Lua file with LuaDoc annotation suitable fopr suggestions/type ahead in VS Code

## Usage
`python ScriptBindingToLua.py [input_file] [output_file]`

### Arguments
- **input_file** _(optional)_: The path to the scriptBinding.xml file to convert. Defaults to "scriptBinding.xml".
- **output_file** _(optional)_: The path to the output Lua file. Defaults to "scriptBinding.lua".


### Float to Number conversion
Currently, all float types is converted to number to avoid warnings in the language server. This can be disabled by changing `SUBSTITUE_FLOAT` to `False`:
```py
SUBSTITUE_FLOAT = False
```

## Example
`python ScriptBindingToLua.py C:\FS25\debugger\scriptBinding.xml C:\MyProjects\scriptBinding.lua`

## Note
The output file will be overwritten if it already exists.

