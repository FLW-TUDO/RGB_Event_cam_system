def find_closest_elements(A, B):
    result = {}

    for a in A:
        closest_b = min(B, key=lambda x: abs(x - a))
        result[a] = closest_b
        B.remove(closest_b)

    return result


def remove_extension_and_convert_to_int(arr):
    # Remove ".png" extension and convert to integers
    modified_arr = [int(file_name[:-4]) for file_name in arr if file_name.endswith('.png')]
    return modified_arr

def remove_delayed_timestamps(result_dict):
    keys_to_remove = []
    for key, value in result_dict.items():
        deviation = abs(int(key) - int(value))

        if deviation > 10000000:
            keys_to_remove.append(key)

    for key in keys_to_remove:
        del result_dict[key]
    return result_dict