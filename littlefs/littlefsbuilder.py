Import("env")
print("Replace MKSPIFFSTOOL with mklittlefs")
env.Replace( MKSPIFFSTOOL=env.get("PROJECT_DIR") + './littlefs/mklittlefs' )
