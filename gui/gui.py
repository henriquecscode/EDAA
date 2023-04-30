# Import necessary packages
import csv
import os
import webbrowser 
import folium
from folium import plugins
import pandas
import rioxarray as rxr
import earthpy as et
import earthpy.spatial as es
from jinja2 import Template
from folium.map import Marker

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

def plotAirport(point):
    '''input: series that contains a numeric named latitude and a numeric named longitude
    this function creates a CircleMarker and adds it to your this_map'''
    popup  = folium.Popup(point[2], max_width=600, max_height=600)
    folium.vector_layers.Marker(location=[point[6], point[7]], tooltip=point[0], popup = popup, marker_color = 'purple').add_to(m)

def plotFlight(edge):
    popup = folium.Popup(edge, max_width=600, max_height=600)
    folium.vector_layers.PolyLine(locations=edge, tooltip=edge[0], popup = popup, color='blue', weight=1, opacity=0.8).add_to(m)



#filename = os.path.expanduser;
with open('../data/airports_actually_used.csv', 'r')  as f:          # Read lines separately
    reader = csv.reader(f, delimiter=',')
    for i, line in enumerate(reader):
        if (i == 0): continue  # Skip header
        plotAirport(line)


## DayofMonth,DayOfWeek,Carrier,OriginAirportID,DestAirportID,DepDelay,ArrDelay,distance,flight_time
airports = pandas.read_csv(filepath_or_buffer='../data/airports_actually_used.csv', sep=',')
airports = pandas.DataFrame(data=airports)[['airport_id', 'lat', 'lng']]


with open('../data/flights.csv', 'r') as f:
    reader = csv.reader(f, delimiter=',')
    count = 0
    for i, line in enumerate(reader):
        if (i == 0): continue
        origin = airports.loc[airports['airport_id'] == int(line[3])].reset_index()
        dest = airports.loc[airports['airport_id'] == int(line[4])].reset_index()
        edge = [[origin['lat'][0], origin['lng'][0]], [dest['lat'][0], dest['lng'][0]]]
        print(count)
        if (count == 5000):
            break;
        count += 1
        plotFlight(edge)


#Set the zoom to the maximum possible
m.fit_bounds(m.get_bounds())

# Display the map
m.save("map.html")
webbrowser.open("map.html")

