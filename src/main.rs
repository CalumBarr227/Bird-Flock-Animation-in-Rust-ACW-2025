#[macro_use]
extern crate glium;
extern crate winit;
extern crate rayon;

use nalgebra::{Matrix4, Perspective3, Point3, Vector3};
use rand::random;
use rayon::prelude::*;
use std::sync::Arc;

const NUM_BIRDS: usize = 10;
const MAX_SPEED: f32 = 0.02;
const NEIGHBOUR_RADIUS: f32 = 1.0;
const SEPARATION_WEIGHT: f32 = 1.5;
const ALIGNMENT_WEIGHT: f32 = 1.0;
const COHESION_WEIGHT: f32 = 1.0;
const GRAVITY: f32 = 0.0005;
const BOUNDARY_SIZE: f32 = 5.0;
const BOUNDARY_FORCE: f32 = 0.1;

#[derive(Clone, Copy)]
struct Bird
{
    position: [f32; 3],
    velocity: [f32; 3],
    acceleration: [f32; 3],
}

impl Bird
{
    fn new() -> Bird
    {
        let pos_x = random::<f32>() * BOUNDARY_SIZE - BOUNDARY_SIZE/2.0;
        let pos_y = random::<f32>() * BOUNDARY_SIZE - BOUNDARY_SIZE/2.0;
        let pos_z = random::<f32>() * BOUNDARY_SIZE - BOUNDARY_SIZE/2.0;
        
        let vel_x = random::<f32>() * 0.02 - 0.01;
        let vel_y = random::<f32>() * 0.02 - 0.01;
        let vel_z = random::<f32>() * 0.02 - 0.01;
        
        Bird
        {
            position: [pos_x, pos_y, pos_z],
            velocity: [vel_x, vel_y, vel_z],
            acceleration: [0.0, 0.0, 0.0],
        }
    }

    fn update(&mut self)
    {
        for i in 0..3
        {
            self.velocity[i] += self.acceleration[i];
            
            let vx = self.velocity[0];
            let vy = self.velocity[1];
            let vz = self.velocity[2];
            let speed = (vx*vx + vy*vy + vz*vz).sqrt();
            
            if speed > MAX_SPEED {
                let scale = MAX_SPEED / speed;
                self.velocity[i] *= scale;
            }
            
            self.position[i] += self.velocity[i];
            
            if self.position[i].abs() > BOUNDARY_SIZE/2.0 {
                self.velocity[i] = -self.velocity[i] * 0.8;
                if self.position[i] > 0.0 {
                    self.position[i] = BOUNDARY_SIZE/2.0;
                } else {
                    self.position[i] = -BOUNDARY_SIZE/2.0;
                }
            }
            
            self.acceleration[i] = 0.0;
        }
    }

    fn apply_force(&mut self, force: [f32; 3])
    {
        self.acceleration[0] += force[0];
        self.acceleration[1] += force[1];
        self.acceleration[2] += force[2];
    }

    fn distance_to(&self, other: &Bird) -> f32 {
        let dx = self.position[0] - other.position[0];
        let dy = self.position[1] - other.position[1];
        let dz = self.position[2] - other.position[2];
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();
        return distance;
    }
}

struct Flock
{
    birds: Vec<Bird>,
}

impl Flock
{
    fn new() -> Flock
    {
        let mut birds = Vec::new();
        
        for i in 0..NUM_BIRDS 
        {
            let bird = Bird::new();
            birds.push(bird);
        }
        
        Flock { birds }
    }

    fn update(&mut self)
    {
        let birds_copy = self.birds.clone();
        let birds_shared = Arc::new(birds_copy);
        
        self.birds.par_iter_mut().for_each(|bird| {
            let mut separation = [0.0, 0.0, 0.0];
            let mut alignment = [0.0, 0.0, 0.0];
            let mut cohesion = [0.0, 0.0, 0.0];
            let mut neighbour_count = 0;

            for other in birds_shared.iter() 
            {
                let bird_ptr = bird as *const _ as usize;
                let other_ptr = other as *const _ as usize;
                
                if bird_ptr == other_ptr
                {
                    continue;
                }

                let dist = bird.distance_to(other);

                if dist < NEIGHBOUR_RADIUS
                 {
                    separation[0] += (bird.position[0] - other.position[0]);
                    separation[1] += (bird.position[1] - other.position[1]);
                    separation[2] += (bird.position[2] - other.position[2]);

                    alignment[0] += other.velocity[0];
                    alignment[1] += other.velocity[1];
                    alignment[2] += other.velocity[2];

                    cohesion[0] += other.position[0];
                    cohesion[1] += other.position[1];
                    cohesion[2] += other.position[2];

                    neighbour_count += 1;
                }
            }

            if neighbour_count > 0 {
                
                separation[0] *= SEPARATION_WEIGHT;
                separation[1] *= SEPARATION_WEIGHT;
                separation[2] *= SEPARATION_WEIGHT;

                alignment[0] = alignment[0] / neighbour_count as f32;
                alignment[1] = alignment[1] / neighbour_count as f32;
                alignment[2] = alignment[2] / neighbour_count as f32;
                alignment[0] = (alignment[0] - bird.velocity[0]) * ALIGNMENT_WEIGHT;
                alignment[1] = (alignment[1] - bird.velocity[1]) * ALIGNMENT_WEIGHT;
                alignment[2] = (alignment[2] - bird.velocity[2]) * ALIGNMENT_WEIGHT;


                cohesion[0] = cohesion[0] / neighbour_count as f32;
                cohesion[1] = cohesion[1] / neighbour_count as f32;
                cohesion[2] = cohesion[2] / neighbour_count as f32;
                cohesion[0] = (cohesion[0] - bird.position[0]) * COHESION_WEIGHT;
                cohesion[1] = (cohesion[1] - bird.position[1]) * COHESION_WEIGHT;
                cohesion[2] = (cohesion[2] - bird.position[2]) * COHESION_WEIGHT;

                bird.apply_force(separation);
                bird.apply_force(alignment);
                bird.apply_force(cohesion);
            }

            bird.apply_force([0.0, -GRAVITY, 0.0]);

            for i in 0..3 {
                if bird.position[i].abs() > BOUNDARY_SIZE/2.0 - 1.0 
                {
                    let boundary_force = -bird.position[i].signum() * BOUNDARY_FORCE;
                    
                    if i == 0 {
                        bird.apply_force([boundary_force, 0.0, 0.0]);
                    } else if i == 1 {
                        bird.apply_force([0.0, boundary_force, 0.0]);
                    } else {
                        bird.apply_force([0.0, 0.0, boundary_force]);
                    }
                }
            }

            bird.update();
        });
    }
}

fn main() {
    #[allow(unused_imports)]
    use glium::{glutin, Surface};

    let event_loop = glium::winit::event_loop::EventLoop::builder()
        .build()
        .expect("event loop building");
    let (window, display) = glium::backend::glutin::SimpleWindowBuilder::new()
        .with_title("Bird Flock Simulation")
        .build(&event_loop);

    let mut flock = Flock::new();

    #[derive(Copy, Clone)]
    struct Vertex {
        position: [f32; 2],
    }

    implement_vertex!(Vertex, position);

    let vertex1 = Vertex { position: [-0.05, -0.0288] };
    let vertex2 = Vertex { position: [ 0.00,  0.0577] };
    let vertex3 = Vertex { position: [ 0.05, -0.0288] };
    let shape = vec![vertex1, vertex2, vertex3];

    let vertex_buffer = glium::VertexBuffer::new(&display, &shape).unwrap();
    let indices = glium::index::NoIndices(glium::index::PrimitiveType::TrianglesList);

    let vertex_shader_src = r#"
        #version 140

        in vec2 position;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        void main() {
            gl_Position = projection * view * model * vec4(position, 0.0, 1.0);
        }
    "#;

    let fragment_shader_src = r#"
        #version 140

        out vec4 color;

        void main() {
            color = vec4(1.0, 0.0, 0.0, 1.0);
        }
    "#;

    let program = glium::Program::from_source(&display, vertex_shader_src, fragment_shader_src, None).unwrap();

    #[allow(deprecated)] 
    let _ = event_loop.run(move |event, window_target| {
        match event {
            winit::event::Event::WindowEvent { event, .. } => match event {

                winit::event::WindowEvent::CloseRequested => window_target.exit(),

                winit::event::WindowEvent::Resized(window_size) => {
                    display.resize(window_size.into());
                },

                winit::event::WindowEvent::RedrawRequested => {
                    let next_frame_time = std::time::Instant::now() + std::time::Duration::from_nanos(16_666_667);
                    winit::event_loop::ControlFlow::WaitUntil(next_frame_time);

                    flock.update();

                    let mut target = display.draw();

                    target.clear_color(0.0, 0.0, 0.0, 1.0);

                    let perspective = Perspective3::new(1.0, std::f32::consts::FRAC_PI_3, 0.1, 100.0);
                    let projection_matrix: [[f32; 4]; 4] = *perspective.as_matrix().as_ref();

                    let eye = Point3::new(0.0, 0.0, 5.0); 
                    let look = Point3::new(0.0, 0.0, 0.0);  
                    let up = Vector3::new(0.0, 1.0, 0.0);  
                    let view_matrix: [[f32; 4]; 4] = *Matrix4::look_at_rh(&eye, &look, &up).as_ref();

                    for bird in &flock.birds {
                        let model_matrix = [
                            [1.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0],
                            [bird.position[0], bird.position[1], bird.position[2], 1.0],
                        ];

                        let uniforms = uniform! {
                            model: model_matrix,
                            view: view_matrix,
                            projection: projection_matrix,
                        };

                        target.draw(&vertex_buffer, &indices, &program, &uniforms, &Default::default()).unwrap();
                    }

                    target.finish().unwrap();
                },
                _ => (),
            },                
            winit::event::Event::AboutToWait => {
                window.request_redraw();
            },
            _ => (),
        };
    });
}
