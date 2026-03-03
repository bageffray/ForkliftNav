using UnityEngine;

/// <summary>
/// Represents a node in a tree structure for pathfinding algorithms like RRT.
/// Each node contains a 3D position and a reference to its parent node.
/// </summary>
public class TreeNode
{
    public TreeNode(Vector3 position, TreeNode parent = null)
    {
        this.Position = position;
        this.Parent = parent;
    }

    public TreeNode(Vector2 position2D, TreeNode parent = null)
    {
        this.Position = new Vector3(position2D.x, 0, position2D.y);
        this.Parent = parent;
    }

    public Vector3 Position;
    public TreeNode Parent;
}