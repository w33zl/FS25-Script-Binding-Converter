# FS25 Script Binding Converter

Converts a scriptBinding.xml file to a Lua file with LuaDoc annotation suitable fopr suggestions/type ahead in VS Code

![GitHub Downloads (all assets, all releases)](https://img.shields.io/github/downloads/w33zl/FS25-Script-Binding-Converter/total)

## Usage
`python ScriptBindingToLua.py [input_file] [output_file]`

### Arguments
- **input_file** _(optional)_: The path to the scriptBinding.xml file to convert. Defaults to "scriptBinding.xml".
- **output_file** _(optional)_: The path to the output Lua file. Defaults to "scriptBinding.lua".


### Float to Number conversion
Currently, all `float` types are converted to `number` to avoid warnings in the Lua language server. This can be disabled by changing `SUBSTITUE_FLOAT` to `False`:
```py
SUBSTITUE_FLOAT = False
```

## Example
`python ScriptBindingToLua.py C:\FS25\debugger\scriptBinding.xml C:\MyProjects\scriptBinding.lua`

## Note
The output file will be overwritten if it already exists.

