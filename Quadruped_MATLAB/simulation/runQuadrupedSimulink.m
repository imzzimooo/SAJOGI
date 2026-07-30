function runQuadrupedSimulink()
%runQuadrupedSimulink Build and run the quadruped Simulink model.

    buildQuadrupedSimulinkModel();
    sim('quadruped_simulink_model');
end
