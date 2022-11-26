from vision import Vision
import numpy as np
import cv2
import matplotlib.pyplot as plt





def distance_transform(vertices,nb_obstacles):
    # Compute distance transform
    from scipy.ndimage.morphology import distance_transform_edt as bwdist
    # Create a binary image of the polygon
    img = np.zeros((1400,1400))
    cv2.fillPoly(img, vertices, 1)
    # Compute distance transform
    d = bwdist(img)

    # Rescale and transform distances

    d2 = (d/100.) + 1

    d0 = 2
    nu = 800

    repulsive = nu*((1./d2 - 1/d0)**2);

    repulsive [d2 > d0] = 0

    # Display repulsive potential
    plt.imshow(repulsive, 'gray')
    plt.title ('Repulsive Potential')
    plt.show()
    return repulsive

def attractive_force(goal,start):
    # Compute attractive force
    x, y = np.meshgrid(np.arange(1400), np.arange(1400))
    attractive = 0.5*((x - goal[0])**2 + (y - goal[1])**2) - 0.5*((x - start[0])**2 + (y - start[1])**2)
    # Display attractive potential
    plt.imshow(attractive, 'gray')
    plt.title ('Attractive Potential')
    plt.show()
    return attractive 

def config_space(vertices,nb_obstacles,goal,start):
    # Compute 2D configuration space
    obstacle = np.zeros((1400,1400))
    cv2.fillPoly(obstacle, vertices, 1)

    plt.plot (start[0], start[1], 'ro', markersize=10);
    plt.plot (goal[0], goal[1], 'ro', color='green', markersize=10);
    # show the obstacle
    plt.imshow(obstacle, 'gray')
    plt.title ('Obstacle')
    plt.show()





def GradientBasedPlanner (f, start_coords, end_coords, max_its):
    # GradientBasedPlanner : This function plans a path through a 2D
    # environment from a start to a destination based on the gradient of the
    # function f which is passed in as a 2D array. The two arguments
    # start_coords and end_coords denote the coordinates of the start and end
    # positions respectively in the array while max_its indicates an upper
    # bound on the number of iterations that the system can use before giving
    # up.
    # The output, route, is an array with 2 columns and n rows where the rows
    # correspond to the coordinates of the robot as it moves along the route.
    # The first column corresponds to the x coordinate and the second to the y coordinate

    [gy, gx] = np.gradient(-f)

    route = np.vstack( [np.array(start_coords), np.array(start_coords)] )
    for i in range(max_its):
        current_point = route[-1,:]
#         print(sum( abs(current_point-end_coords) ))
        if sum( abs(current_point-end_coords) ) < 5.0:
            print('Reached the goal !');
            break
        ix = int(round( current_point[1] ))
        iy = int(round( current_point[0] ))
        vx = gx[ix, iy]
        vy = gy[ix, iy]
        dt = 1 / np.linalg.norm([vx, vy])
        next_point = current_point + dt*np.array( [vx, vy] )
        route = np.vstack( [route, next_point] )
    route = route[1:,:]
        
    return route


def gradient_visualization(f):
    # Compute attractive force
    fig = plt.figure(figsize=(12,8))
    # two axis
    ax = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)
    [gx, gy] = np.gradient(-f)
    ax.imshow(gy, 'gray')
    ax.set_title ('Gx=df/dx - gradient')
    ax2.imshow(gx, 'gray')
    ax2.set_title ('Gy=df/dy - gradient')
    plt.show()
    return gx,gy

def gradient_plot(x,y, gx,gy, skip=10):
    plt.figure(figsize=(12,8))
    Q = plt.quiver(x[::skip, ::skip], y[::skip, ::skip], gx[::skip, ::skip], gy[::skip, ::skip],
                   pivot='mid', units='inches')
    qk = plt.quiverkey(Q, 0.9, 0.9, 1, r'$1 \frac{m}{s}$', labelpos='E',
                       coordinates='figure')



if __name__=="__main__":
    vis_obj = Vision()
    img = cv2.imread('src/sample_image.png')
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    filtered_img = cv2.bilateralFilter(img,9,75,75)
    # get obstacle position
    vertices, nb_obstacles = vis_obj.get_obstacle_position(filtered_img, 0)
    # plot the vertices
    for i in range(len(vertices)):
        for j in range(len(vertices[i])):
            cv2.circle(img, (vertices[i][j][0][0], vertices[i][j][0][1]), 5, (0, 255, 0), -1)
    # show the image
    plt.imshow(img)
    plt.show()
    goal = [1200, 50]
    start = [50, 350]
    repulsive = distance_transform(vertices,nb_obstacles)
    config_space(vertices,nb_obstacles,goal,start)
    # inital position of the robot
    initial_thymio_pos = np.array([[0,0],[0,0]]) # will be replaced
    # goal position of the obstacles 
    attractive = attractive_force(goal,start)
    
    f = attractive + repulsive
    route = GradientBasedPlanner(f, start, goal, 100000)
    gx,gy=gradient_visualization(f)

    skip = 10
    x, y = np.meshgrid(np.arange(1400), np.arange(1400))
    xidx = np.arange(0,1400,skip)
    yidx = np.arange(0,1400,skip)

    gradient_plot(x,y, gy,gx, skip=10)

    plt.plot(start[0], start[1], 'ro', markersize=10);
    plt.plot(goal[0], goal[1], 'ro', color='green', markersize=10);
    plt.plot(route[:,0], route[:,1], linewidth=3);
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Gradient-based planner')
    plt.show()


