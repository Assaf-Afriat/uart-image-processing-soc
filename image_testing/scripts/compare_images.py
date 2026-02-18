"""Quick script to compare input vs received image and find divergence point."""

input_file = r"C:\Users\afria\OneDrive\Desktop\Cursor\Design\Final Project v00\image_testing\img_input\input_image.pgm"
received_file = r"C:\Users\afria\OneDrive\Desktop\Cursor\Design\Final Project v00\image_testing\img_output\received_image_20260214_190602.ppm"

def parse_ppm(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()
    # Skip header (P3, dimensions, max)
    # Data starts at line 3 (0-indexed)
    pixels = []
    for line in lines[3:]:
        vals = line.strip().split()
        for i in range(0, len(vals), 3):
            if i+2 < len(vals):
                pixels.append((int(vals[i]), int(vals[i+1]), int(vals[i+2])))
    return pixels

print("Loading input image...")
inp = parse_ppm(input_file)
print(f"  Input pixels: {len(inp)}")

print("Loading received image...")
rcv = parse_ppm(received_file)
print(f"  Received pixels: {len(rcv)}")

# Find first difference
first_diff = None
diff_count = 0
for i in range(min(len(inp), len(rcv))):
    if inp[i] != rcv[i]:
        if first_diff is None:
            first_diff = i
        diff_count += 1

row = first_diff // 256 if first_diff else None
col = first_diff % 256 if first_diff else None

print(f"\nFirst difference at pixel {first_diff} (row {row}, col {col})")
print(f"Total differing pixels: {diff_count} / {min(len(inp), len(rcv))}")

if first_diff is not None:
    # Show context around first difference
    start = max(0, first_diff - 4)
    end = min(len(inp), first_diff + 8)
    print(f"\nPixels around first difference (pixel {first_diff}):")
    print(f"  {'Idx':>6}  {'Row':>3}  {'Col':>3}  {'Input':>18}  {'Received':>18}  {'Match':>5}")
    for i in range(start, end):
        r, c = i // 256, i % 256
        match = "OK" if inp[i] == rcv[i] else "DIFF"
        print(f"  {i:6d}  {r:3d}  {c:3d}  {str(inp[i]):>18}  {str(rcv[i]):>18}  {match}")

    # Check if it's a shift - does received[i] == input[i + offset]?
    print(f"\nChecking for shift pattern...")
    for offset in range(-8, 9):
        if offset == 0:
            continue
        matches = 0
        check_range = range(first_diff, min(first_diff + 100, len(inp), len(rcv)))
        for i in check_range:
            j = i + offset
            if 0 <= j < len(inp) and rcv[i] == inp[j]:
                matches += 1
        pct = matches * 100 / len(check_range) if check_range else 0
        if pct > 50:
            print(f"  Offset {offset:+d}: {pct:.0f}% match ({matches}/{len(check_range)})")

    # Show row-by-row diff summary
    print(f"\nRow-by-row diff count:")
    for row_num in range(256):
        row_diffs = 0
        for col_num in range(256):
            idx = row_num * 256 + col_num
            if idx < len(inp) and idx < len(rcv) and inp[idx] != rcv[idx]:
                row_diffs += 1
        if row_diffs > 0:
            print(f"  Row {row_num:3d}: {row_diffs:3d} pixels differ")
            if row_num > row + 20:  # Only show first 20 affected rows
                print(f"  ... (more rows differ)")
                break

input("\nPress Enter to exit...")
