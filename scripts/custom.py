import numpy as np
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import scipy.ndimage as img


def custom_process(height):
    """ Custom function for experimental data analysis. """

    return height


def fuzzy_custom(height, growth, canopy):
    """ Perform fuzzy logic analysis on data. """
    stress_data = np.empty_like(height)

    # Create inputs/outputs to the fuzzy control system
    h_var = ctrl.Antecedent(np.linspace(-0.01, 1.01, num=100), 'height')
    g_var = ctrl.Antecedent(np.linspace(-0.01, 1.01, num=100), 'growth')
    c_var = ctrl.Antecedent(np.linspace(-0.01, 1.01, num=100), 'canopy')
    stress_var = ctrl.Consequent(np.linspace(0, 1, num=100), 'stress')

    # Create membership functions for Antecedents and Consequents
    g_var.automf(3)
    c_var.automf(3)

    h_var['poor'] = fuzz.trapmf(np.linspace(-0.01, 1.01, num=100),
                                [0, 0, 0.25, 0.5])
    h_var['average'] = fuzz.trimf(np.linspace(-0.01, 1.01, num=100), [0.25,
                                                                      0.5,
                                                                      0.75])
    h_var['good'] = fuzz.trapmf(np.linspace(-0.01, 1.01, num=100), [0.5, 0.75,
                                                                    1, 1])

    stress_var['low'] = fuzz.trapmf(np.linspace(-0.01, 1.01, num=100),
                                    [0, 0, 0.25, 0.5])
    stress_var['med'] = fuzz.trimf(np.linspace(-0.01, 1.01, num=100),
                                   [0.25, 0.5, 0.75])
    stress_var['high'] = fuzz.trapmf(np.linspace(-0.01, 1.01, num=100),
                                     [0.5, 0.75, 1, 1])

    # Create basic rule-set grouping poor performance with high stress

    # Low height rules
    #     GROWTH
    # H     L M H
    # E    ______
    # I  L| L L M
    # G  M| L L L
    # H  H| L L L
    # T
    rule_l1 = ctrl.Rule((h_var['poor'] & g_var['good'] & c_var['good']),
                        stress_var['med'])
    rule_l2 = ctrl.Rule((h_var['poor'] & g_var['poor']), stress_var['high'])
    rule_l3 = ctrl.Rule((h_var['poor'] & g_var['average']), stress_var['high'])
    rule_l4 = ctrl.Rule((h_var['poor'] & c_var['poor']), stress_var['high'])
    rule_l5 = ctrl.Rule((h_var['poor'] & c_var['average']), stress_var['high'])

    # Med height rules
    #     GROWTH
    # H     L M H
    # E    ______
    # I  L| M M H
    # G  M| M M M
    # H  H| L M M
    # T
    rule_m1 = ctrl.Rule((h_var['average'] & g_var['good'] & c_var['good']),
                        stress_var['low'])
    rule_m2 = ctrl.Rule((h_var['average'] & g_var['poor'] & c_var['poor']),
                        stress_var['high'])
    rule_m3 = ctrl.Rule((h_var['average'] & g_var['average']),
                        stress_var['med'])
    rule_m4 = ctrl.Rule((h_var['average'] & c_var['average']),
                        stress_var['med'])
    rule_m5 = ctrl.Rule((h_var['average'] & g_var['poor'] & c_var['good']),
                        stress_var['med'])
    rule_m6 = ctrl.Rule((h_var['average'] & g_var['good'] & c_var['poor']),
                        stress_var['med'])

    # High height rule
    #     GROWTH
    # H     L M H
    # E    ______
    # I  L| H H H
    # G  M| H H H
    # H  H| H H H
    # T
    rule_h1 = ctrl.Rule((h_var['good']), stress_var['low'])

    # Create the fuzzy control system with the defined rule-set
    stress_sys = ctrl.ControlSystem([rule_l1, rule_l2, rule_l3, rule_l4,
                                     rule_l5, rule_m1, rule_m2, rule_m3,
                                     rule_m4, rule_m5, rule_m6, rule_h1])
    stress_sim = ctrl.ControlSystemSimulation(stress_sys)

    for i in range(height.shape[2]):

        # Split off layer from rest of data
        height_layer = height[:, :, i]
        growth_layer = growth[:, :, i]
        canopy_layer = canopy[:, :, i]

        # Normalize all of the data
        height_layer = np.divide(height_layer, np.amax(height_layer))
        growth_layer = np.divide(growth_layer, np.amax(growth_layer))
        canopy_layer = np.divide(canopy_layer, np.amax(canopy_layer))

        # Input each data map into the simulation
        stress_sim.input['height'] = height_layer
        stress_sim.input['growth'] = growth_layer
        stress_sim.input['canopy'] = canopy_layer

        # Run the simulation and extract the output
        stress_sim.compute()
        stress_layer = stress_sim.output['stress']

        # Smooth out local minimums and maximums
        stress_layer = img.grey_opening(stress_layer, structure=np.ones((3, 3)))

        stress_data[:, :, i] = stress_layer

    return stress_data
