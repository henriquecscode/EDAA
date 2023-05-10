# Import necessary packages
import csv
import enum
import random
import os
import webbrowser 
import folium
from folium import plugins
import pandas


class AirportType(enum.Enum):
    ORIGIN = 1
    DESTINATION = 2
    MIDDLE = 3

# # Modify Marker template to include the onClick event
# click_template = """{% macro script(this, kwargs) %}
#     var {{ this.get_name() }} = L.marker(
#         {{ this.location|tojson }},
#         {{ this.options|tojson }}
#     ).addTo({{ this._parent.get_name() }}).on('click', onClick);
# {% endmacro %}"""

# # Change template to custom template
# Marker._template = Template(click_template)

# click_js = """function onClick(e) {
#                  var point = e.latlng; alert(point)
#                  }"""
                 
# e = folium.Element(click_js)


m = folium.Map(location=[40.0150, -105.2705], prefer_canvas=True)

# html = m.get_root()
# html.script.get_root().render()
# html.script._children[e.get_name()] = e

## Start: green
## End: dark red
## Middle: cadet blue
## Edges: light gray

colors =  ['beige', 'black', 'blue', 'cadetblue', 'darkblue', 'darkgreen', 'darkpurple', 'darkred', 'gray', 'green', 'lightblue', 'lightgray', 'lightgreen', 'lightred', 'orange', 'pink', 'purple', 'red'];

def plotAirport(point, type):
    '''input: series that contains a numeric named latitude and a numeric named longitude
    this function creates a CircleMarker and adds it to your this_map'''
    popup  = folium.Popup(point[0], max_width=600, max_height=600)

    if (type == AirportType.ORIGIN):
        folium.vector_layers.Marker(location=[point[6], point[7]], tooltip=point[2], popup = popup, icon=folium.Icon(icon='plane', color='green')).add_to(m)
    elif (type == AirportType.DESTINATION):
        folium.vector_layers.Marker(location=[point[6], point[7]], tooltip=point[2], popup = popup, icon=folium.Icon(icon='plane', color='darkred')).add_to(m)
    else:
        folium.vector_layers.Marker(location=[point[6], point[7]], tooltip=point[2], popup = popup, icon=folium.Icon(icon='plane', color='cadetblue')).add_to(m)

def plotFlight(edge):
    popup = folium.Popup(edge, max_width=600, max_height=600)
    folium.vector_layers.PolyLine(locations=edge, tooltip=edge[0], popup = popup, color='lightgray', weight=1, opacity=0.8).add_to(m)



filename = os.path.expanduser('data/airports_actually_used.csv'); ##TODO: file path to use.
with open(filename, 'r')  as f:          # Read lines separately
    reader = csv.reader(f, delimiter=',')
    length = len(f.readlines())

    for i, line in enumerate(reader):
        if (i == 0): 
            continue
        elif (i == 1):  # Skip header
            plotAirport(line, AirportType.ORIGIN)
        elif (i == length - 1):
            plotAirport(line, AirportType.DESTINATION)
        else:
            plotAirport(line, AirportType.MIDDLE)


# DayofMonth,DayOfWeek,Carrier,OriginAirportID,DestAirportID,DepDelay,ArrDelay,distance,flight_time
# airports = pandas.read_csv(filepath_or_buffer='data/airports_actually_used.csv', sep=',')
# airports = pandas.DataFrame(data=airports)[['airport_id', 'lat', 'lng']]


# with open('data/flights.csv', 'r') as f: ##TODO: file path to use - now this should be a cmd-line arg.
#     reader = csv.reader(f, delimiter=',')
#     count = 0
#     for i, line in enumerate(reader):
#         if (i == 0):
#             continue
#         origin = airports.loc[airports['airport_id'] == int(line[3])].reset_index()
#         dest = airports.loc[airports['airport_id'] == int(line[4])].reset_index()
#         edge = [[origin['lat'][0], origin['lng'][0]], [dest['lat'][0], dest['lng'][0]]]
#         print(count)
#         if (count == 5000):
#             break;
#         count += 1
#         plotFlight(edge)


#Set the zoom to the maximum possible
m.fit_bounds(m.get_bounds())
folium.LayerControl().add_to(m)
plugins.MiniMap().add_to(m)

# Display the map
m.save("map.html")
webbrowser.open("map.html")

