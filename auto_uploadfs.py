Import("env")

def after_upload(source, target, env):
    print("\n--- [Auto-UploadFS] Code upload finished! Starting filesystem upload... ---")
    # This invokes the PlatformIO CLI to target 'uploadfs' specifically for the active environment
    env.Execute(f"platformio run --target uploadfs -e {env['PIOENV']}")

# Register the callback to trigger after the standard "upload" action finishes successfully
env.AddPostAction("upload", after_upload)