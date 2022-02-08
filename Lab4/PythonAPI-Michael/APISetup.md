# How to use the Python API
In order to use the python api you must follow these steps:
Make sure you have following files in your directory (or in a folder included in the sys path. See "plowExampleWithAPI.py"), in order to run the various examples:
1. sim.py
2. simConst.py
3. the appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux)

Then, first run the CoppeliaSim with the content of "ChildScript.txt" in your model's child script.  
Scene_Hirarchy -> "yourmodel" -> Add -> Associated Child Script -> Non-Threaded.

Fianlly, run your Python script built off "exampleScript.py" and it will connect to CoppeliaSim.
