#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 13 18:14:19 2024

@author: dkreitler
"""

import cv2
print(f"Loading {__file__}")


def add_cross(image_path, cross_color=(0, 0, 255), center=None):
    # Load the image
    image = cv2.imread(image_path)
    # Get the center coordinates
    if center:
        center_x, center_y = center[0], center[1]
    else:
        height, width, _ = image.shape
        center_x, center_y = width // 2, height // 2

    # Define the size of the cross and its color
    cross_size = 20

    # Draw the horizontal line of the cross
    cv2.line(
        image,
        (center_x - cross_size, center_y),
        (center_x + cross_size, center_y),
        cross_color,
        2
    )

    # Draw the vertical line of the cross
    cv2.line(
        image,
        (center_x, center_y - cross_size),
        (center_x, center_y + cross_size),
        cross_color,
        2
    )

    # Save the result
    cv2.imwrite(image_path, image)


def add_text_bottom_left(image_path, *args, text_color=(0, 0, 255)):
    # Load the image
    image = cv2.imread(image_path)

    # Get the height and width of the image
    height, width, _ = image.shape

    # Define font and other text-related parameters
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_thickness = 2

    # Calculate the position for the first text
    initial_x = 10
    initial_y = height - 10  # Start from the bottom

    # Iterate through the text list and place each text on the image
    for text in args:
        # Put text on the image
        cv2.putText(image, text, (initial_x, initial_y), font,
                    font_scale, text_color, font_thickness, cv2.LINE_AA)

        # Update the y-coordinate for the next text
        initial_y -= 30  # You can adjust the spacing between lines as needed

    # Save the result
    cv2.imwrite(image_path, image)
