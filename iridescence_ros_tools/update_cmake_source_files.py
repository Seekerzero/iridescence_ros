import os

# Path to the directory containing your source files
source_dir = "./src"  # Adjust this as needed
# Path to your CMakeLists.txt file
cmake_file = "./CMakeLists.txt"

# Get all .cpp files in the source directory
cpp_files = []
for root, dirs, files in os.walk(source_dir):
    for file in files:
        if file.endswith(".cpp"):
            # Store the relative path of the .cpp files
            cpp_files.append(os.path.relpath(os.path.join(root, file)))

# Read the current CMakeLists.txt file
with open(cmake_file, "r") as f:
    cmake_content = f.readlines()

# We will modify the CMakeLists.txt by searching for the section with source files
new_content = []
in_source_list = False

for line in cmake_content:
    # Check if we are in the section where the source files are listed
    if line.strip().startswith("set(SOURCE_FILES"):
        in_source_list = True
        new_content.append("set(SOURCE_FILES\n")
        for cpp_file in cpp_files:
            new_content.append(f"    {cpp_file}\n")
        new_content.append(")\n")
    elif in_source_list and line.strip().startswith(")"):
        # End of the source files section
        in_source_list = False
    elif not in_source_list:
        # If we are not in the source file section, copy the line as is
        new_content.append(line)

# Write the updated content back to the CMakeLists.txt file
with open(cmake_file, "w") as f:
    f.writelines(new_content)

print(f"Updated CMakeLists.txt with {len(cpp_files)} source files.")
