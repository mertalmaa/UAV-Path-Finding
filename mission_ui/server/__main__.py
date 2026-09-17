from .app import main

if __name__ == "__main__":  # guard required: the planner worker process uses spawn
    main()
