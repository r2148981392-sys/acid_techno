import os
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np


# 1. Load Maps

ACIDITY_MAP_PATH = "resource/AcidityMap.jpg"
GENERATED_MAP_PATH = "resource/GeneratedMap.jpg"

if not os.path.exists(ACIDITY_MAP_PATH) or not os.path.exists(GENERATED_MAP_PATH):
    print(f"ERROR: {ACIDITY_MAP_PATH} or {GENERATED_MAP_PATH} not found.")
    exit(1)

acidity_map = mpimg.imread(ACIDITY_MAP_PATH)
if len(acidity_map.shape) == 3:
    acidity_map_gray = acidity_map[:, :, 0]
else:
    acidity_map_gray = acidity_map

generated_map = mpimg.imread(GENERATED_MAP_PATH)
if len(generated_map.shape) == 3:
    generated_map_gray = generated_map[:, :, 0]
else:
    generated_map_gray = generated_map

# Ensure both maps have the same dimensions before comparison
if acidity_map_gray.shape != generated_map_gray.shape:
    print(f"ERROR: Map dimensions do not match! Acidity Map: {acidity_map_gray.shape}, Generated Map: {generated_map_gray.shape}")
    exit(1)

acidity_map_height, acidity_map_width = acidity_map_gray.shape

# Convert pixel values to pH scale
true_ph_matrix = 6.0 + acidity_map_gray * (3.0 / 255.0)
generated_ph_matrix = 6.0 + generated_map_gray * (3.0 / 255.0)


# 2. Quantifying the Amount of Difference

# Calculate the exact amount of difference for EACH pixel color/pH
error_matrix = np.abs(true_ph_matrix - generated_ph_matrix)

# Statistical breakdown of the amount of difference
mae = np.mean(error_matrix)
max_error = np.max(error_matrix)
min_error = np.min(error_matrix)

# Categorize pixels by the AMOUNT of their discrepancy
total_pixels = acidity_map_width * acidity_map_height
low_diff = np.sum(error_matrix < 0.1)        # Tiny difference (< 0.1 pH)
med_diff = np.sum((error_matrix >= 0.1) & (error_matrix < 0.5))  # Noticeable (0.1 - 0.5 pH)
high_diff = np.sum(error_matrix >= 0.5)      # Large difference (>= 0.5 pH)

print(f"\n--- Analysis of Pixel Value Differences ---")
print(f"Total Pixels Analyzed          : {total_pixels}")
print(f"Average Amount of Difference   : {mae:.4f} pH (Mean Absolute Error)")
print(f"Maximum Amount of Difference   : {max_error:.4f} pH")
print(f"Minimum Amount of Difference   : {min_error:.4f} pH")
print(f"----------------------------------------")
print(f"Pixels with tiny diff (<0.1 pH) : {low_diff} ({(low_diff/total_pixels)*100:.2f}%)")
print(f"Pixels with med diff (0.1-0.5) : {med_diff} ({(med_diff/total_pixels)*100:.2f}%)")
print(f"Pixels with large diff (>=0.5) : {high_diff} ({(high_diff/total_pixels)*100:.2f}%)")
print(f"----------------------------------------")


# 3. Visualization

fig, axes = plt.subplots(1, 3, figsize=(18, 5))

im1 = axes[0].imshow(true_ph_matrix, cmap='gray', vmin=6.0, vmax=9.0, origin='upper')
axes[0].set_title("Standard Acidity Map (True pH)")
fig.colorbar(im1, ax=axes[0], label="pH Scale")

im2 = axes[1].imshow(generated_ph_matrix, cmap='gray', vmin=6.0, vmax=9.0, origin='upper')
axes[1].set_title("Generated Map")
fig.colorbar(im2, ax=axes[1], label="pH Scale")

# The colorbar on the third map perfectly shows the "amount of difference" per pixel
im3 = axes[2].imshow(error_matrix, cmap='Reds', vmin=0.0, vmax=2.0, origin='upper')
axes[2].set_title(f"Amount of Difference Map (MAE: {mae:.3f})")
fig.colorbar(im3, ax=axes[2], label="pH Difference Amount")

for ax in axes:
    ax.set_xlabel("Image X (pixels)")
    ax.set_ylabel("Image Y (pixels)")

plt.tight_layout()
print("Displaying comparison plots...")
plt.show()