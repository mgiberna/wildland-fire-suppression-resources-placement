'''
Author: Marco Giberna
Email: gibimarco@gmail.com
python3.9.9
'''
import pickle
from d_generation import generate_dataset
from placement_of_wildland_fire_suppression_resources_gurobi import optimize_and_write_model
from ILS.ILS import test_ILS

if __name__ == '__main__':
    #CREATION OF THE DATASET
    #PARAMETERS
    sizes = (6,10,20,30)
    propagation_times = (((7,9),(2,4),(4,6),(6,8)),
                         ((7,9),(1,3),(4,6),(6,8)),
                         ((7,9),(2,4),(3,5),(6,8)),
                         ((7,9),(1,3),(3,5),(6,8)),
                         ((7,9),(2,4),(4,6),(4,6)),
                         ((7,9),(1,3),(4,6),(4,6)),
                         ((7,9),(2,4),(3,5),(3,5)),
                         ((7,9),(1,3),(3,5),(3,5)))
    #generating the dataset
    dataset = generate_dataset(sizes, propagation_times)
    #dumping the dataset in a pickle
    with open('dataset.pickle', 'wb') as file:
        pickle.dump(dataset, file)
    #OPTIMISATION WITH GUROBI
    optimize_and_write_model(dataset, setup=True, use_reduced_dataset=False)  # 5 resources
    with open('dataset.pickle', 'rb') as file:  #open it again to get a "clean" dataset
        dataset = pickle.load(file)
    optimize_and_write_model(dataset, setup=False, use_reduced_dataset=False)  # 6 resources
    #OPTIMISATION WITH ILS HEURISTIC
    test_ILS(dataset, n_resources=5, delta=50, time_horizon=28, n_replications=5, show_plot=False)
    test_ILS(dataset, n_resources=6, delta=50, time_horizon=28, n_replications=5, show_plot=False)



