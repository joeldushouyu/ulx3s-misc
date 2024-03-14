filename = "whiteScreen-640-400.mem"
width = 640
height = 400

# Open the file for writing
with open(filename, 'w') as f:
    # Write the header
    f.write("@0\n")

    # Write the pixel values
    for _ in range(height):
        line = "ffffff\n" * width
        f.write(line)

print(f"Content has been written to {filename}")
