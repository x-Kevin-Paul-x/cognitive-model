from graphviz import Digraph
import os

def create_flowchart():
    """Creates a flowchart of the project methodology."""
    dot = Digraph(comment='Project Methodology')
    dot.attr(rankdir='TB', size='8,5')

    dot.node('A', 'Start')
    dot.node('B', 'Define Robot Structure (URDF)')
    dot.node('C', 'Set up Simulation Environment (PyBullet)')
    dot.node('D', 'Load Robot and Object')
    dot.node('E', 'Execute Pick-and-Place Task')
    dot.node('F', 'Capture Screenshots')
    dot.node('G', 'Generate Report')
    dot.node('H', 'End')

    dot.edge('A', 'B')
    dot.edge('B', 'C')
    dot.edge('C', 'D')
    dot.edge('D', 'E')
    dot.edge('E', 'F')
    dot.edge('F', 'G')
    dot.edge('G', 'H')

    # Save the flowchart
    output_filename = 'flowchart'
    dot.render(output_filename, format='png', cleanup=True)
    print(f"Flowchart saved as {output_filename}.png")

if __name__ == '__main__':
    # Change to the script's directory to save the flowchart there
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    create_flowchart()
