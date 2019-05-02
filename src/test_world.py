import world
import numpy
import itertools
import exploration
import random

weights = [0.7, 0.3]
scale_factors = [81.3816, 1.0]


latitude, longitude = 37.29069,-107.84629
width, height, spacing, orientation = 100, 100, 2.0, (numpy.pi/7)

w = world.World(latitude, longitude, width, height, spacing, orientation)
print "world created"

w.add_measurement(0, (37.2907347334,-107.8462555), 0.64)
w.update()
print "model updated"

current_cell = w.cell_corresponding_to_gps(37.2907347334,-107.8462555)
print current_cell
plan_window = [5, 5]
x_values = range(max(0, current_cell[0] - plan_window[0] / 2), min(w.width, current_cell[0] + plan_window[0] / 2))
y_values = range(max(0, current_cell[1] - plan_window[1] / 2), min(w.height, current_cell[1] + plan_window[1] / 2))
candidate_locations = [x_values, y_values]
candidate_locations = list(itertools.product(*candidate_locations))
e = exploration.Evaluation(weights, scale_factors)
best_candidate_indices = e.evaluate((latitude, longitude), candidate_locations, w)
#print candidate_locations[best_index]
best_candidate_index = random.choice(best_candidate_indices)
best_candidate = w.gps_corresponding_to_cell(candidate_locations[best_candidate_index][0],candidate_locations[best_candidate_index][1])
print  best_candidate
