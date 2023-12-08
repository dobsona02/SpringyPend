using Godot;
using System;

public partial class pendulum_1 : Node3D
{
	MeshInstance3D Anchor;
	MeshInstance3D Ball;
	SpringModel spring;
	Label XCoord;
	Label YCoord;
	Label ZCoord;
	Label KE;
	Label PE;
	Label TotalEnergy;

	SpringyPendSim pend;
    Simulator sim;
	
	double xA, yA, zA; // coords of anchor
	float length; // length of pendulum
	float Lo;
	double angle; // pendulum angle
	double angleInit; // initial pendulum angle
	double time;
	double[] Energy;

	Vector3 endA; // end point of anchor
	Vector3 endB; // end point for pendulum bob

 
	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		GD.Print("In Godot");
        xA = 0.0; yA = 1.2; zA = 0.0;
        Anchor = GetNode<MeshInstance3D>("Anchor");
        Ball = GetNode<MeshInstance3D>("Ball");
        spring = GetNode<SpringModel>("SpringModel");
        XCoord = GetNode<Label>("XCoord");
        YCoord = GetNode<Label>("YCoord");
        ZCoord = GetNode<Label>("ZCoord");
        KE = GetNode<Label>("KE");
        PE = GetNode<Label>("PE");
        TotalEnergy = GetNode<Label>("Total Energy");
        Energy = new double[3]; //0 is PE, 1 is KE, 2 is Total Energy

        endA = new Vector3((float)xA, (float)yA, (float)zA);
        Anchor.Position = endA; //Set initial position of anchor in Godot

        pend = new SpringyPendSim();

        Lo = length = 0.9f;
        spring.GenMesh(0.05f, 0.015f, length, 6.0f, 62);

        endB.X = 0.4f;
        endB.Y = 1.2f;
        endB.Z = 0.4f;
        PlacePendulum(endB);

        pend.XCoord = endB.X;
        pend.YCoord = endB.Y;
        pend.ZCoord = endB.Z;

        time = 0.0;

	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		//Position Information    
        XCoord.Text = endB.X.ToString("0.00");
        YCoord.Text = endB.Y.ToString("0.00");
        ZCoord.Text = endB.Z.ToString("0.00");

        //Get the energy
        Energy = pend.CalcEnergy(endA);
        

        // //Display the energy
        PE.Text = Energy[0].ToString("0.00");
        KE.Text = Energy[1].ToString("0.00");
        TotalEnergy.Text = Energy[2].ToString("0.00");

        //pend.StepRK2(time, delta, endA);
        pend.StepRK4(time, delta, endA);

        
        endB.X = (float)pend.XCoord;
        endB.Y = (float)pend.YCoord;
        endB.Z = (float)pend.ZCoord;
        PlacePendulum(endB);
        time += delta;

	}

    // public override void _PhysicsProcess(double delta)
    // {
    //     base._PhysicsProcess(delta);

    //     sim.Step(time, delta);
    // }


	private void PlacePendulum(Vector3 endBB)
    {
        Ball.Position = endBB;
        spring.PlaceEndPoints(endA, endB);
    }

}