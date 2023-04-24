import numpy as np
import matplotlib.pyplot as plt
import utils
import json

if __name__ == '__main__':
    path = utils.get_path('Map_stayaway.csv')
    with open('positions.json', 'r') as f:
        actual = json.load(f)

    path_rows = [point[0] for point in path]
    path_cols = [point[1] for point in path]
    actual_rows = [point[0] for point in actual]
    actual_cols = [point[1] for point in actual]
    plt.scatter(path_rows, path_cols, s=8, c='purple', marker='x', label='Desired')
    plt.scatter(actual_rows, actual_cols, s=8, c='black', marker='D', label='Actual')
    plt.legend()
    plt.title('Desired vs. Actual Path')
    plt.show()
