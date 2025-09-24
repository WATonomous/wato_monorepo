# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import numpy as np

class BoxConstraint:
    """
    Bounded constraints lb <= x <= ub as polytopic constraints -Ix <= -b and Ix <= b. np.vstack(-I, I) forms the H matrix from III-D-b of the paper
    """

    def __init__(self, lb=None, ub=None, plot_idxs=None):
        """
        :param lb: dimwise list of lower bounds.
        :param ub: dimwise list of lower bounds.
        :param plot_idxs: When plotting, the box itself might be defined in some dimension greater than 2 but we might only want to
        plot the workspace variables and so plot_idxs allows us to limit the consideration of plot_constraint_set to those variables.
        """
        self.lb = np.array(lb, ndmin=2).reshape(-1, 1)
        self.ub = np.array(ub, ndmin=2).reshape(-1, 1)
        self.plot_idxs = plot_idxs
        self.dim = self.lb.shape[0]
        assert (self.lb < self.ub).all(), (
            "Lower bounds must be greater than corresponding upper bound for any given dimension"
        )
        self.setup_constraint_matrix()

    def __str__(self):
        return "Lower bound: %s, Upper bound: %s" % (self.lb, self.ub)

    def get_random_vectors(self, num_samples):
        rand_samples = np.random.rand(self.dim, num_samples)
        for i in range(self.dim):
            scale_factor, shift_factor = (self.ub[i] - self.lb[i]), self.lb[i]
            rand_samples[i, :] = (rand_samples[i, :] * scale_factor) + shift_factor
        return rand_samples

    def setup_constraint_matrix(self):
        dim = self.lb.shape[0]
        # Casadi can't do matrix mult with Torch instances but only numpy instead. So have to use the np version of the H and b matrix/vector when
        # defining constraints in the opti stack.
        self.H_np = np.vstack((-np.eye(dim), np.eye(dim)))
        #self.H = torch.Tensor(self.H_np)
        # self.b = torch.Tensor(np.hstack((-self.lb, self.ub)))
        self.b_np = np.vstack((-self.lb, self.ub))
        #self.b = torch.Tensor(self.b_np)
        # print(self.b)
        self.sym_func = lambda x: self.H_np @ np.array(x, ndmin=2).T - self.b_np

    def check_satisfaction(self, sample):
        # If sample is within the polytope defined by the constraints return 1 else 0.
        # print(sample, np.array(sample, ndmin=2).T, self.sym_func(sample), self.b)
        return (self.sym_func(sample) <= 0).all()

    def generate_uniform_samples(self, num_samples):
        n = int(np.round(num_samples ** (1.0 / self.lb.shape[0])))

        # Generate a 1D array of n equally spaced values between the lower and
        # upper bounds for each dimension
        coords = []
        for i in range(self.lb.shape[0]):
            coords.append(np.linspace(self.lb[i, 0], self.ub[i, 0], n))

        # Create a meshgrid of all possible combinations of the n-dimensions
        meshes = np.meshgrid(*coords, indexing="ij")

        # Flatten the meshgrid and stack the coordinates to create an array of
        # size (K, n-dimensions)
        samples = np.vstack([m.flatten() for m in meshes])

        # Truncate the array to K samples
        samples = samples[:num_samples, :]

        # Print the resulting array
        return samples

    def clip_to_bounds(self, samples):
        return np.clip(samples, self.lb, self.ub)
