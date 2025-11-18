# Contributing to the Satellite Thruster Control System

Thank you for your interest in this project!

## 📌 Read-Only Reference Implementation

**This repository is provided as-is as a reference implementation for educational and research use.**

### What This Means:

- ❌ **Pull requests are NOT accepted** - This is a read-only archive
- ❌ **Issues are DISABLED** - No bug tracking here
- ✅ **Fork and modify freely** - Adapt for your own research
- ✅ **GitHub Discussions ENABLED** - Ask questions, share experiences
- 📧 **Email for urgent matters**: ofjord99@gmail.com

### This Repository is NOT Actively Monitored

I (the author) have graduated and am not actively maintaining this repository. While I may occasionally check in, please do not expect timely responses.

---

## How to Use This Project

### Option 1: Download and Use As-Is

1. **Download** the repository (Clone or Download ZIP)
2. **Follow** the [README.md](../README.md) installation instructions
3. **Run** simulations or adapt for your hardware
4. **Reference** the extensive documentation

### Option 2: Fork and Modify for Your Research

This is the recommended approach if you want to make changes:

1. **Fork** this repository to your GitHub account
2. **Clone** your fork to your local machine
3. **Modify** freely for your specific needs:
   - Adapt physics parameters for your testbed
   - Change MPC formulation
   - Add new mission types
   - Integrate different hardware
4. **Maintain** your own version independently

### Option 3: Use for Learning/Reference

- Study the MPC implementation in `mpc.py`
- Learn from the mission architecture
- Understand hardware integration patterns
- Use as teaching material

---

## Getting Help

### Documentation (Your First Stop)

Read the comprehensive documentation included:
- [README.md](../README.md) - Quick start and overview
- [ARCHITECTURE.md](../ARCHITECTURE.md) - System design and module structure
- [DEVELOPMENT_GUIDE.md](../DEVELOPMENT_GUIDE.md) - Code organization and patterns
- [TROUBLESHOOTING.md](../TROUBLESHOOTING.md) - Common issues and solutions
- [TESTING_GUIDE.md](../TESTING_GUIDE.md) - How to run and write tests
- [HARDWARE_TEST_PROCEDURE.md](../HARDWARE_TEST_PROCEDURE.md) - Hardware setup and safety
- [LESSONS_LEARNED.md](../LESSONS_LEARNED.md) - Development insights and tips

### GitHub Discussions (Community Support)

Have questions? Use [GitHub Discussions](https://github.com/AevarOfjord/mpc-air-bearing-satellite/discussions):
- Ask implementation questions
- Share your adaptations
- Discuss MPC strategies
- Get help from the community

**Note**: I may not respond, but other users might help!

### External Resources

- **Gurobi Support**: https://support.gurobi.com/
- **OptiTrack Support**: https://v21.optitrack.com/support/
- **Python/MPC Forums**: Stack Overflow, Reddit r/ControlTheory

### Urgent Matters Only

For truly urgent matters (security issues, critical bugs affecting safety):
📧 **Email**: ofjord99@gmail.com

**Please note**: I may not respond quickly. This is a reference implementation, not a supported product.

---

## What You Can Do With This Code

### Encouraged Uses:

- ✅ **Academic Research**: Use as a foundation for your satellite control research
- ✅ **Education**: Teach MPC, control systems, or robotics with this example
- ✅ **Industry Prototyping**: Adapt for commercial satellite testbed projects
- ✅ **Benchmarking**: Compare your control algorithms against this implementation
- ✅ **Learning**: Study how MPC works in a real system

### Examples of Adaptations:

- Port to different hardware (different thrusters, sensors, etc.)
- Implement alternative control strategies (LQR, PID, etc.) for comparison
- Add new mission types (rendezvous, docking, formation flying)
- Scale to larger/smaller testbeds
- Integrate with different motion capture systems

### If You Modify This Code:

While you're free to use and modify this code, consider:
- **Document your changes** clearly in your fork
- **Credit this project** in your papers/documentation
- **Share findings** via Discussions (others might benefit!)

---

## Code of Conduct

In GitHub Discussions and any project communication:
- Be respectful and professional
- Welcome diverse perspectives
- Give credit where due
- Focus on constructive feedback
- No harassment or inappropriate behavior

---

## License

This project is licensed under the MIT License. See [LICENSE](../LICENSE) for details.

**By using this code**, you agree to the MIT License terms.

---

## Citation

If you use this code in academic work, please cite:

```bibtex
@mastersthesis{oefjoerd2025satellite,
  title={Model Predictive Control for Satellite Thruster Navigation on Air-Bearing Testbed},
  author={Oefjoerd, Aevar},
  year={2025},
  school={University of Kentucky},
  department={Mechanical Engineering},
  type={Master's Thesis}
}
```

---

Thank you for your interest in this project!

For questions, use [GitHub Discussions](https://github.com/AevarOfjord/mpc-air-bearing-satellite/discussions).
