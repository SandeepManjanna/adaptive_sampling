#!/usr/bin/python2.7

"""model.py: Given samples returns model."""

import random

import numpy

#from sklearn import gaussian_process
#Sandeep Moving to GPy instead of sklearn
import GPy
# TODO new version of sklearn
#from sklearn.gaussian_process import GaussianProcessRegressor

# TODO(Alberto) Consider different models and create abstract class.
class Model(object):
    def __init__(self, num_features=2, num_target=1, model_type='gaussian'):
        self.model_type = model_type
        if model_type == 'gaussian':
            #self.model = gaussian_process.GaussianProcessRegressor() # TODO(alberto) parameters
            #self.model = gaussian_process.GaussianProcess(theta0=1e-2, thetaL=1e-4, thetaU=1e-1) # TODO(alberto) parameters
            #Sandeep: setting model to none for now
            self.model = None
        else:
            self.model = None
        self.input = numpy.empty((0, num_features))
        self.observations = numpy.empty((0, num_target))

    
    def update_observations(self, X, y):
        """Update observations and fit to model.

        Args:
            X: An array with shape (n_samples, n_features) with the input at 
                which observations were made.
            y: An array with shape (n_samples, ) or shape 
                (n_samples, n_targets) with the observations of the 
                output to be predicted.
        Returns:

        Raises:
        """
        if self.input.size == 0:
            self.input = X
        else:
            self.input = numpy.concatenate((self.input, X))
        if self.observations.size == 0:
            self.observations = y
        else:
            self.observations = numpy.concatenate((self.observations, y))
    
    def fit(self, X, y):
        """Fit data observation to model.

        Args:
            X: An array with shape (n_samples, n_features) with the input at 
                which observations were made.
            y: An array with shape (n_samples, ) or shape 
                (n_samples, n_targets) with the observations of the 
                output to be predicted.

        Returns:

        Raises:

        """
        try:
            #Sandeep : Moving to Gpy
            #self.model.fit(X, y)#self.input, self.observations) # TODO handle error if no convergence.
            self.model = GPy.models.SparseGPRegression(numpy.array(X),numpy.array(y),num_inducing=100)
            self.model.optimize()
            #if X.shape[0] > 100: # TODO more principled.
            #    self.model.optimize_restarts(num_restarts=5)
            #else:
            #    self.model.optimize_restarts(num_restarts=10)
            return True
        except:
            return False
        
        
    def predict(self, X, return_std=True, return_cov=False): # TODO according to version.
        """Predict the data.

        Args:
            X: An array with shape (n_eval, n_features) giving the point(s) at 
                which the prediction(s) should be made.
            return_std : bool, default: False
                If True, the standard-deviation of the predictive distribution 
                at the query points is returned along with the mean.
            return_cov : bool, default: False
                If True, the covariance of the joint predictive distribution 
                at the query points is returned along with the mean.

        Returns:
            y_mean : array, shape = (n_samples, [n_output_dims])
                Mean of predictive distribution a query points
            y_std : array, shape = (n_samples,), optional
                Standard deviation of predictive distribution at query points. 
                Only returned when return_std is True.
            y_cov : array, shape = (n_samples, n_samples), optional
                Covariance of joint predictive distribution a query points. 
                Only returned when return_cov is True.

        Raises:

        """
        # TODO check actual model.
        #print "In model::predict()"
        #print X
        if self.model:#self.input.size > 0 and self.observations.size > 0:
            #Sandeep: Moving to Gpy
            #res = self.model.predict(X, return_std)
            res, res_var = self.model.predict(numpy.array(X))
            #res = self.model.predict(X, return_std, return_cov)
            return res, res_var

def main():
    with open('../sample_data/trialData.csv', 'r') as trial_data:
        training_data = [[], []] # 0: input, 1: observations.
        test_data = [[], []] # 0: input, 1: observations.
        for i, l in enumerate(trial_data):
            if i > 0:
                l = map(float, l.split(','))
                if not l[3:5] in training_data[0] and not l[3:5] in test_data[0]:
                    if bool(random.getrandbits(1)):
                        training_data[0].append(l[3:5])
                        training_data[1].append(l[5])
                    else:
                        test_data[0].append(l[3:5])
                        test_data[1].append(l[5])
                if i > 300:
                    break
        m = Model()
        print numpy.array(training_data[0]).shape
        print numpy.array(training_data[1]).shape
        m.update_observations(numpy.array(training_data[0]), 
          numpy.array(training_data[1])) 
        if m.fit(numpy.array(training_data[0]), numpy.array(training_data[1])): # TODO(alberto) clean update_obseravtions
            Y, Y_std = m.predict(numpy.array(test_data[0]))
            for y, y_std, ground_truth in zip(Y, Y_std, test_data[1]):
                if abs(y - ground_truth) > 0.0001: # TODO(alberto) parameter.
                    print y, y_std, ground_truth
        
          
       
if __name__ == '__main__':
    main()
