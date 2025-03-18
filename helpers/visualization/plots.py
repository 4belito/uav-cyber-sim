

import plotly.express as px
import plotly.graph_objects as go

def plot_3d_interactive(markers, title="title"):
    # Create a list to store all waypoints
    data = []

    for label, marker_set in markers.items():
        x, y, z = zip(*marker_set.get("pos"))  # Extract positions
        color = marker_set.get("color")  # Default color

        # Add a scatter plot for each marker set
        trace = go.Scatter3d(
            x=x, y=y, z=z,
            mode="markers",
            marker=dict(size=6, color=color),
            name=label
        )
        data.append(trace)

    # Create figure with all markers
    fig = go.Figure(data=data)
    fig.update_layout(
        title=dict(
            text=title,
            x=0.5,  # Centers the title
            xanchor="center"),
        scene=dict(
            xaxis_title="x",
            yaxis_title="y",
            zaxis_title="z",
            zaxis=dict(range=[0, 10])),
        width=800,  # Adjust figure size
        height=600
    )

    fig.show()  # Display the interactive plot