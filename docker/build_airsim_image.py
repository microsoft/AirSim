import argparse
import subprocess

def main():
    parser = argparse.ArgumentParser(description='AirSim docker image builder')
    parser.add_argument('--source', action='store_true', help='compile unreal and airsim from source') # default is false
    parser.add_argument('--base_image', type=str, help='base image name AND tag', required=True)
    parser.add_argument('--target_image', type=str, help='base image name AND tag')

    args = parser.parse_args()
    build_docker_image(args)

def build_docker_image(args):
    target_image_tag = args.base_image.split(":")[1] # take tag from base image
    dockerfile = 'Dockerfile_source'
    if args.source:
        if not args.target_image:
            args.target_image = 'airsim_source' + ':' + target_image_tag

    else:
        dockerfile = 'Dockerfile_binary'
        if not args.target_image:
            args.target_image = 'airsim_binary' + ':' + target_image_tag

    docker_command = ['docker', 'build', '--network=host', '-t', args.target_image, '-f',  dockerfile, '--build-arg', 'BASE_IMAGE=' + args.base_image, '.']
    print " ".join(docker_command)
    subprocess.call(docker_command)

if __name__=="__main__":
    main()