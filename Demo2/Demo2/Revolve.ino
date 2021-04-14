    case REVOLVE:
      if (currentTime - startTime >= (2 * PI * radius)/rho_dot_set) {
        current_state = STOP;
      }
      else if (currentTime - startTime < (2 * PI * radius)/rho_dot_set) {
        run_phi_controller = true;
        rho_dot_set = 0.25;
        phi_dot_set = rho_dot_set / radius;
      }
      break;
