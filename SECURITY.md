# Security Policy

## Academic Project Notice

This is an academic research project. This repository is provided as-is for educational and research purposes.

## Security Best Practices for Users

### 1. Protect Your Credentials

**NEVER commit sensitive files to version control:**

- ✅ `gurobi.lic` is in `.gitignore` - keep it that way
- ✅ Never commit `.env` files
- ✅ Never commit API keys or passwords
- ✅ Use environment variables for secrets

### 2. Gurobi License Security

Your Gurobi academic license file contains sensitive credentials:

```bash
# Good - License in project root (gitignored)
/path/to/project/gurobi.lic  ✅

# Bad - License in repository
git add gurobi.lic  ❌ NEVER DO THIS
```

If you accidentally commit `gurobi.lic`:

1. **Immediately revoke** the license at https://www.gurobi.com/
2. **Remove from git history** using `git filter-repo`
3. **Generate a new license**
4. **Verify** `.gitignore` includes `gurobi.lic`

### 3. Hardware Safety

When using real hardware:

- ✅ Always test in simulation first
- ✅ Verify position bounds are configured correctly
- ✅ Keep emergency stop accessible
- ✅ Never disable safety checks in production
- ✅ Ensure proper workspace clearance

## Using This Project

This software is provided under the MIT License. Users are responsible for:
- Following security best practices outlined above
- Testing thoroughly before any hardware deployment
- Understanding the risks of controlling physical systems
- Implementing additional safety measures as needed for their use case

**Remember:** This is educational software for learning about satellite control systems. Always prioritize safety when working with physical hardware.
