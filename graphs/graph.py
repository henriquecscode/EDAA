import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
import numpy as np
import pandas as pd

def flatten_list(_2d_list):
    flat_list = []
    # Iterate through the outer list
    for element in _2d_list:
        if type(element) is list:
            # If the element is of type list, iterate through the sublist
            for item in element:
                flat_list.append(float(item))
        else:
            flat_list.append(element)
    return flat_list

# Read data from CSV file
for i in ['bfs_by_node.csv', 'bfs.csv', 'dfs_by_node.csv', 'dfs.csv', 'dijkstra_by_node.csv', 'dijkstra.csv', 'spanning.csv', "erdos_by_node.csv", "erdos.csv"]:
    data = pd.read_csv(i)
    
    # Extract the x and y values
    x = data['Sample']
    y1 = data['DepDelayFilter 1']
    y2 = data['DepDelayFilter 2']
    y3 = data['DepDelayFilter 3']
    y4 = data['DepDelayFilter 4']
    y5 = data['DepDelayFilter 5']
    y6 = data['No Filter']
    
    # Create a new figure
    fig = plt.figure()
    
    # Add a subplot
    ax = fig.add_subplot(1, 1, 1)
    
    # Plot the lines
    ax.plot(x, y1, label='DepDelayFilter 1', marker='o')
    ax.plot(x, y2, label='DepDelayFilter 2', marker='o')
    ax.plot(x, y3, label='DepDelayFilter 3', marker='o')
    ax.plot(x, y4, label='DepDelayFilter 4', marker='o')
    ax.plot(x, y5, label='DepDelayFilter 5', marker='o')
    ax.plot(x, y6, label='No Filter', marker='o')
    
    y_values = [y1.to_list(), y2.to_list(), y3.to_list(), y4.to_list(), y5.to_list(), y6.to_list()]
    y_values = flatten_list(y_values)
    y_values.sort()
    y_values = [round(x, 4) for x in y_values]
    print('length: ', len(y_values))
    
    ax.locator_params(axis='both', nbins=4)
    ax.set_yticklabels(y_values)
    
    
    # Add legend
    ax.legend()
    
    # Set x and y axis labels
    ax.set_xlabel('Sample Size')
    ax.set_ylabel('Performance (s)')

    plt.tight_layout()
    
    # Show the plot
    photo = i.replace('.csv', '') + '.png'
    plt.savefig(photo)
    