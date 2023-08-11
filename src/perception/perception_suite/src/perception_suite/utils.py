def calculate_iou(box1, box2):
    
    # Calculate the coordinates of the intersection rectangle
    x_left = max(box1.min_x, box2.min_x)
    x_right = min(box1.max_x, box2.max_x)
    y_top = max(box1.min_y, box2.min_y)
    y_bottom = min(box1.max_y, box2.max_y)
    
    # Calculate the area of the intersection rectangle
    if x_right < x_left or y_bottom < y_top:
        return 0
    intersection_area = (x_right - x_left) * (y_bottom - y_top)
    
    # Calculate the area of each box
    box1_area = (box1.max_x - box1.min_x) * (box1.max_y - box1.min_y)
    box2_area = (box2.max_x - box2.min_x) * (box2.max_y - box2.min_y)
    
    # Calculate the Union area
    union_area = box1_area + box2_area - intersection_area
    
    # Calculate IoU
    iou = intersection_area / union_area
    return iou

