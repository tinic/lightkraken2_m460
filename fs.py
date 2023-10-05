import sys
import io
import os

from FATtools.Volume import vopen, vclose, copy_tree_in
from FATtools.mkfat import exfat_mkfs
from FATtools.disk import disk

numBytes = int(sys.argv[1])
scanDir = os.path.normpath(sys.argv[2])
headerFile = sys.argv[3]
imageFile = sys.argv[4]

print(scanDir)

def bytesio_to_c_header(bytes_io_obj, var_name="data", line_length=120):
    # Getting bytes from BytesIO
    byte_data = bytes_io_obj.getvalue()
    
    # Convert bytes to string of hex values
    hex_values = ["0x{:02X}".format(b) for b in byte_data]
    
    # Split hex values into lines
    lines = []
    line = ""
    for hex_value in hex_values:
        if len(line) + len(hex_value) + 2 > line_length:  # +2 for ", "
            lines.append(line)
            line = ""
        line += hex_value + ", "
    if line:
        lines.append(line)
    
    # Create C header content
    header_content = [
        "#ifndef _FS_H_",
        "#define _FS_H_",
        "",
        "unsigned char {}[{}] = {{".format(var_name, len(bytes_io_obj.getvalue()))
    ]
    header_content.extend(lines)
    header_content.append("};")
    header_content.append("")
    header_content.append("#endif  // #ifndef _FS_H_")
    
    return "\n".join(header_content)

if numBytes<128000:
    print("exFat much be at least 128000 bytes")
    exit()

img = io.BytesIO(numBytes*b'\x00')

vol = vopen(img, 'r+b', what='disk')
exfat_mkfs(vol, vol.size)
vclose(vol)

vol = vopen(img, 'r+b')
copy_tree_in(scanDir, vol)
vclose(vol)

open(headerFile, 'w').write(bytesio_to_c_header(img, "fs_data"))
open(imageFile, 'wb').write(img.getvalue())
